/*!\file ci2c.c
** \author SMFSW
** \version 1.2
** \copyright MIT SMFSW (2017)
** \brief arduino master i2c in plain c code
** \warning Don't access (r/w) last 16b internal address byte alone right after init, this would lead to hazardous result (in such case, make a dummy read of addr 0 before)
**/

// TODO: add interrupt vector / callback for it operations (if not too messy)
// TODO: consider interrupts at least for RX when slave (and TX when master)

#include "ci2c.h"

#define START          0x08
#define REPEATED_START      0x10
#define MT_SLA_ACK        0x18
#define MT_SLA_NACK       0x20
#define MT_DATA_ACK       0x28
#define MT_DATA_NACK      0x30
#define MR_SLA_ACK        0x40
#define MR_SLA_NACK       0x48
#define MR_DATA_ACK       0x50
#define MR_DATA_NACK      0x58
#define LOST_ARBTRTN      0x38
#define TWI_STATUS        (TWSR & 0xF8)

//#define isSetRegBit(r, b)   ((r & (1 << b)) != 0)
//#define isClrRegBit(r, b)   ((r & (1 << b)) == 0)

#define setRegBit(r, b)     r |= (1 << b)       //!< set bit \b b in register \b r
#define clrRegBit(r, b)     r &= (uint8_t) (~(1 << b))  //!< clear bit \b b in register \b r
#define invRegBit(r, b)     r ^= (1 << b)       //!< invert bit \b b in register \b r

/*!\struct i2c
** \brief static ci2c bus config and control parameters
**/
static struct {
  /*!\struct cfg
  ** \brief ci2c bus parameters
  **/
  struct {
    I2C_SPEED speed;      //!< i2c bus speed
    uint8_t   retries;    //!< i2c message retries when fail
    uint16_t  timeout;    //!< i2c timeout (ms)
  } cfg;
  uint16_t    start_wait;   //!< time start waiting for acknowledge
  bool      busy;     //!< true if already busy (in case of interrupts implementation)
} i2c = { { (I2C_SPEED) 0, DEF_CI2C_NB_RETRIES, DEF_CI2C_TIMEOUT }, 0, false };


// Needed prototypes
static bool I2C_wr(I2C_SLAVE * slave, const uint16_t reg_addr, uint8_t * data, const uint16_t bytes);
static bool I2C_rd(I2C_SLAVE * slave, const uint16_t reg_addr, uint8_t * data, const uint16_t bytes);


/*!\brief Init an I2C slave structure for cMI2C communication
** \param [in] slave - pointer to the I2C slave structure to init
** \param [in] sl_addr - I2C slave address
** \param [in] reg_sz - internal register map size
** \return nothing
**/
void I2C_slave_init(I2C_SLAVE * slave, const uint8_t sl_addr, const I2C_INT_SIZE reg_sz)
{
  while(xSemaphoreTake(i2cSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  //(void) I2C_slave_set_addr(slave, sl_addr);
  slave->cfg.addr = sl_addr;
  
  //(void) I2C_slave_set_reg_size(slave, reg_sz);
  slave->cfg.reg_size = reg_sz > I2C_16B_REG ? I2C_16B_REG : reg_sz;

  //I2C_slave_set_rw_func(slave, (ci2c_fct_ptr) I2C_wr, I2C_WRITE);
  ci2c_fct_ptr * pfcw = (ci2c_fct_ptr*) (I2C_WRITE ? &slave->cfg.rd : &slave->cfg.wr);
  *pfcw = I2C_wr;
  
  //I2C_slave_set_rw_func(slave, (ci2c_fct_ptr) I2C_rd, I2C_READ);
  ci2c_fct_ptr * pfcr = (ci2c_fct_ptr*) (I2C_READ ? &slave->cfg.rd : &slave->cfg.wr);
  *pfcr = I2C_rd;
  
  slave->reg_addr = (uint16_t) -1;  // To be sure to send address on first access (warning: unless last 16b byte address is accessed alone)
  slave->status = I2C_OK;
  xSemaphoreGive(i2cSemaphore); // Release the semaphore 
}

/*!\brief Set I2C current register address
** \attribute inline
** \param [in, out] slave - pointer to the I2C slave structure
** \param [in] reg_addr - register address
** \return nothing
**/
static inline void __attribute__((__always_inline__)) I2C_slave_set_reg_addr(I2C_SLAVE * slave, const uint16_t reg_addr) {
  slave->reg_addr = reg_addr; }



/*!\brief Enable I2c module on arduino board (including pull-ups,
 *        enabling of ACK, and setting clock frequency)
** \param [in] speed - I2C bus speed in KHz
** \return nothing
**/
void I2C_init(const uint16_t speed)
{
  while(xSemaphoreTake(i2cSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  // Set SDA and SCL to ports with pull-ups
  setRegBit(PORTC, 4);
  setRegBit(PORTC, 5);

  //(void) I2C_set_speed(speed);
  i2c.cfg.speed = (I2C_SPEED) ((speed == 0) ? (uint16_t) I2C_STD : ((speed > (uint16_t) I2C_FM) ? (uint16_t) I2C_FM : speed));
  clrRegBit(TWCR, TWEN);  // Ensure i2c module is disabled,
  // Set prescaler and clock frequency
  clrRegBit(TWSR, TWPS0);
  clrRegBit(TWSR, TWPS1);
  TWBR = (((F_CPU / 1000) / i2c.cfg.speed) - 16) / 2;

  // i2c reset
  TWCR = 0; // re-enable module 
  setRegBit(TWCR, TWEA);
  setRegBit(TWCR, TWEN);  
  
  xSemaphoreGive(i2cSemaphore); // Release the semaphore 
}

/*!\brief Disable I2c module on arduino board (releasing pull-ups, and TWI control)
** \return nothing
**/
void I2C_uninit()
{
  while(xSemaphoreTake(i2cSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  // Release SDA and SCL ports pull-ups
  clrRegBit(PORTC, 4);
  clrRegBit(PORTC, 5);

  TWCR = 0;
  xSemaphoreGive(i2cSemaphore); // Release the semaphore 
}


/*!\brief Change I2C ack timeout
** \param [in] timeout - I2C ack timeout (500 ms max)
** \return Configured timeout
**/
uint16_t I2C_set_timeout(const uint16_t timeout)
{
  while(xSemaphoreTake(i2cSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  static const uint16_t max_timeout = 500;
  i2c.cfg.timeout = (timeout > max_timeout) ? max_timeout : timeout;
  xSemaphoreGive(i2cSemaphore); // Release the semaphore 
  return i2c.cfg.timeout;
}

/*!\brief Change I2C message retries (in case of failure)
** \param [in] retries - I2C number of retries (max of 8)
** \return Configured number of retries
**/
uint8_t I2C_set_retries(const uint8_t retries)
{
  while(xSemaphoreTake(i2cSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  static const uint16_t max_retries = 8;
  i2c.cfg.retries = (retries > max_retries) ? max_retries : retries;
  xSemaphoreGive(i2cSemaphore); // Release the semaphore 
  return i2c.cfg.retries;
}

/*!\brief This function writes the provided data to the address specified.
** \param [in, out] slave - pointer to the I2C slave structure
** \param [in] reg_addr - register address in register map
** \param [in] data - pointer to the first byte of a block of data to write
** \param [in] bytes - indicates how many bytes of data to write
** \return I2C_STATUS status of write attempt
**/
I2C_STATUS I2C_write(I2C_SLAVE * slave, const uint16_t reg_addr, uint8_t * data, const uint16_t bytes) {
  while(xSemaphoreTake(i2cSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  I2C_STATUS output;
  const I2C_RW rw = I2C_WRITE;
  uint8_t     retry = i2c.cfg.retries;
  bool      ack = false;
  ci2c_fct_ptr  fc = (ci2c_fct_ptr) (rw ? slave->cfg.rd : slave->cfg.wr);

  if (i2c.busy)  { output = slave->status = I2C_BUSY; xSemaphoreGive(i2cSemaphore); return output;  }
  i2c.busy = true;

  ack = fc(slave, reg_addr, data, bytes);
  while ((!ack) && (retry != 0))  // If com not successful, retry some more times
  {
    delay(5);
    ack = fc(slave, reg_addr, data, bytes);
    retry--;
  }

  i2c.busy = false;
  output = slave->status = ack ? I2C_OK : I2C_NACK;
  xSemaphoreGive(i2cSemaphore); // Release the semaphore 
  return output;
}

/*!\brief This function reads data from the address specified and stores this
 *        data in the area provided by the pointer.
** \param [in, out] slave - pointer to the I2C slave structure
** \param [in] reg_addr - register address in register map
** \param [in, out] data - pointer to the first byte of a block of data to read
** \param [in] bytes - indicates how many bytes of data to read
** \return I2C_STATUS status of read attempt
**/
I2C_STATUS I2C_read(I2C_SLAVE * slave, const uint16_t reg_addr, uint8_t * data, const uint16_t bytes) {
  while(xSemaphoreTake(i2cSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  I2C_STATUS output;
  const I2C_RW rw = I2C_READ;
  uint8_t     retry = i2c.cfg.retries;
  bool      ack = false;
  ci2c_fct_ptr  fc = (ci2c_fct_ptr) (rw ? slave->cfg.rd : slave->cfg.wr);

  if (i2c.busy)  { output = slave->status = I2C_BUSY; xSemaphoreGive(i2cSemaphore); return output; }
  i2c.busy = true;

  ack = fc(slave, reg_addr, data, bytes);
  while ((!ack) && (retry != 0))  // If com not successful, retry some more times
  {
    delay(5);
    ack = fc(slave, reg_addr, data, bytes);
    retry--;
  }

  i2c.busy = false;
  output = slave->status = ack ? I2C_OK : I2C_NACK;
  xSemaphoreGive(i2cSemaphore); // Release the semaphore 
  return output;
  
}


/*!\brief This procedure calls appropriate functions to perform a proper send transaction on I2C bus.
** \param [in, out] slave - pointer to the I2C slave structure
** \param [in] reg_addr - register address in register map
** \param [in] data - pointer to the first byte of a block of data to write
** \param [in] bytes - indicates how many bytes of data to write
** \return Boolean indicating success/fail of write attempt
**/
static bool I2C_wr(I2C_SLAVE * slave, const uint16_t reg_addr, uint8_t * data, const uint16_t bytes)
{
  
  while(xSemaphoreTake(i2cSemaphore, (TickType_t) 10) == pdFALSE) {} // Wait on semaphore
  if (bytes == 0)                       { xSemaphoreGive(i2cSemaphore); return false; }

  bool i2cStart = false;
  i2c.start_wait = (uint16_t) millis();

  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  while (!(TWCR & (1 << TWINT))) { 
    if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
        TWCR = 0; // i2c reset
        setRegBit(TWCR, TWEA);
        setRegBit(TWCR, TWEN);
        i2cStart = false; 
        break;
      }
  }

  if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START))  { i2cStart = true; }
  if (TWI_STATUS == LOST_ARBTRTN){
    TWCR = 0; // i2c reset
    setRegBit(TWCR, TWEA);
    setRegBit(TWCR, TWEN);
  }

  if (i2cStart == false)                 { xSemaphoreGive(i2cSemaphore); return false; }

      I2C_RW rw = I2C_WRITE;
      bool i2csndaddr = false;
      bool i2csndaddrset = false;
      TWDR = (slave->cfg.addr << 1) | rw;
    
      i2c.start_wait = (uint16_t) millis();
    
      TWCR = (1 << TWINT) | (1 << TWEN);
    
      while (!(TWCR & (1 << TWINT)))
      { if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
        TWCR = 0; // i2c reset
        setRegBit(TWCR, TWEA);
        setRegBit(TWCR, TWEN);
        i2csndaddr = false; i2csndaddrset = true; break; } }
    
      if(!i2csndaddrset) {
    
        if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK)) { i2csndaddr = true; i2csndaddrset = true; }
    
        if(!i2csndaddrset) {
    
          if ((TWI_STATUS == MT_SLA_NACK) || (TWI_STATUS == MR_SLA_NACK)) {
              i2c.start_wait = (uint16_t) millis();
        
              TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
        
              while ((TWCR & (1 << TWSTO))) {
                if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
                  TWCR = 0; // i2c reset
                  setRegBit(TWCR, TWEA);
                  setRegBit(TWCR, TWEN);
                  break;
                }
              }
            }
          else                              {
            TWCR = 0; // i2c reset
            setRegBit(TWCR, TWEA);
            setRegBit(TWCR, TWEN);
            }
        }
    
      }

  
  if (i2csndaddr == false)         { xSemaphoreGive(i2cSemaphore); return false; }
  if ((slave->cfg.reg_size) && (reg_addr != slave->reg_addr)) // Don't send address if writing next
  {
    (void) I2C_slave_set_reg_addr(slave, reg_addr);

    if (slave->cfg.reg_size >= I2C_16B_REG) // if size >2, 16bit address is used
    {
        uint8_t dat = ((uint8_t) (reg_addr >> 8));
        bool i2cwr8 = false;
        TWDR = dat;
      
        i2c.start_wait = (uint16_t) millis();
      
        TWCR = (1 << TWINT) | (1 << TWEN);
      
        while (!(TWCR & (1 << TWINT))) { 
          if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
            TWCR = 0; // i2c reset
            setRegBit(TWCR, TWEA);
            setRegBit(TWCR, TWEN);
            i2cwr8 = false;
            break;
          }
        }
      
        if (TWI_STATUS == MT_DATA_ACK)    { i2cwr8 = true; }
      
        if (TWI_STATUS == MT_DATA_NACK){
          i2c.start_wait = (uint16_t) millis();
        
          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
        
          while ((TWCR & (1 << TWSTO))) {
            if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
              TWCR = 0; // i2c reset
              setRegBit(TWCR, TWEA);
              setRegBit(TWCR, TWEN);
              break;
            }
          }
        } else {
          TWCR = 0; // i2c reset
          setRegBit(TWCR, TWEA);
          setRegBit(TWCR, TWEN);
        }
      if (i2cwr8 == false)  { xSemaphoreGive(i2cSemaphore); return false; }
    }

        uint8_t dat = ((uint8_t) reg_addr);
        bool i2cwr8 = false;
        TWDR = dat;
      
        i2c.start_wait = (uint16_t) millis();
      
        TWCR = (1 << TWINT) | (1 << TWEN);
      
        while (!(TWCR & (1 << TWINT))) { 
          if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
            TWCR = 0; // i2c reset
            setRegBit(TWCR, TWEA);
            setRegBit(TWCR, TWEN);
            i2cwr8 = false;
            break;
          }
        }
      
        if (TWI_STATUS == MT_DATA_ACK)    { i2cwr8 = true; }
      
        if (TWI_STATUS == MT_DATA_NACK){
          i2c.start_wait = (uint16_t) millis();
        
          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
        
          while ((TWCR & (1 << TWSTO))) {
            if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
              TWCR = 0; // i2c reset
              setRegBit(TWCR, TWEA);
              setRegBit(TWCR, TWEN);
              break;
            }
          }
        } else {
          TWCR = 0; // i2c reset
          setRegBit(TWCR, TWEA);
          setRegBit(TWCR, TWEN);
        }
    if (i2cwr8 == false)       { xSemaphoreGive(i2cSemaphore); return false; }
  }

  for (uint16_t cnt = 0; cnt < bytes; cnt++)
  {
      uint8_t dat = (*data++);
      bool i2cwr8 = false;
      TWDR = dat;
    
      i2c.start_wait = (uint16_t) millis();
    
      TWCR = (1 << TWINT) | (1 << TWEN);
    
      while (!(TWCR & (1 << TWINT))) { 
        if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
          TWCR = 0; // i2c reset
          setRegBit(TWCR, TWEA);
          setRegBit(TWCR, TWEN);
          i2cwr8 = false;
          break;
        }
      }
    
      if (TWI_STATUS == MT_DATA_ACK)    { i2cwr8 = true; }
    
      if (TWI_STATUS == MT_DATA_NACK){
        i2c.start_wait = (uint16_t) millis();
      
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
      
        while ((TWCR & (1 << TWSTO))) {
          if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
            TWCR = 0; // i2c reset
            setRegBit(TWCR, TWEA);
            setRegBit(TWCR, TWEN);
            break;
          }
        }
      } else {
        TWCR = 0; // i2c reset
        setRegBit(TWCR, TWEA);
        setRegBit(TWCR, TWEN);
      }
    
    if (i2cwr8 == false)              { xSemaphoreGive(i2cSemaphore); return false; }
    slave->reg_addr++;
  }

  bool i2cStop = true;
  i2c.start_wait = (uint16_t) millis();

  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

  while ((TWCR & (1 << TWSTO))) {
    if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
      TWCR = 0; // i2c reset
      setRegBit(TWCR, TWEA);
      setRegBit(TWCR, TWEN);
      i2cStop =  false;
      break;
    }
  }
  
  if (i2cStop == false)                  { xSemaphoreGive(i2cSemaphore); return false; }
  xSemaphoreGive(i2cSemaphore);
  return true;
}


/*!\brief This procedure calls appropriate functions to perform a proper receive transaction on I2C bus.
** \param [in, out] slave - pointer to the I2C slave structure
** \param [in] reg_addr - register address in register map
** \param [in, out] data - pointer to the first byte of a block of data to read
** \param [in] bytes - indicates how many bytes of data to read
** \return Boolean indicating success/fail of read attempt
**/
static bool I2C_rd(I2C_SLAVE * slave, const uint16_t reg_addr, uint8_t * data, const uint16_t bytes)
{
  if (bytes == 0)                         {  return false; }

  if ((slave->cfg.reg_size) && (reg_addr != slave->reg_addr)) // Don't send address if reading next
  {
    (void) I2C_slave_set_reg_addr(slave, reg_addr);

    bool i2cStart = false;
    i2c.start_wait = (uint16_t) millis();
  
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  
    while (!(TWCR & (1 << TWINT))) { 
      if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
          TWCR = 0; // i2c reset
          setRegBit(TWCR, TWEA);
          setRegBit(TWCR, TWEN);
          i2cStart = false; 
          break;
        }
    }
  
    if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START))  { i2cStart = true; }
    if (TWI_STATUS == LOST_ARBTRTN){
      TWCR = 0; // i2c reset
      setRegBit(TWCR, TWEA);
      setRegBit(TWCR, TWEN);
    }
    
    if (i2cStart == false)                 { return false; }

        I2C_RW rw = I2C_WRITE;
        bool i2csndaddr = false;
        bool i2csndaddrset = false;
        TWDR = (slave->cfg.addr << 1) | rw;
      
        i2c.start_wait = (uint16_t) millis();
      
        TWCR = (1 << TWINT) | (1 << TWEN);
      
        while (!(TWCR & (1 << TWINT)))
        { if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
          TWCR = 0; // i2c reset
          setRegBit(TWCR, TWEA);
          setRegBit(TWCR, TWEN);
          i2csndaddr = false; i2csndaddrset = true; break; } }
      
        if(!i2csndaddrset) {
      
          if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK)) { i2csndaddr = true; i2csndaddrset = true; }
      
          if(!i2csndaddrset) {
      
            if ((TWI_STATUS == MT_SLA_NACK) || (TWI_STATUS == MR_SLA_NACK)) {
                i2c.start_wait = (uint16_t) millis();
          
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
          
                while ((TWCR & (1 << TWSTO))) {
                  if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
                    TWCR = 0; // i2c reset
                    setRegBit(TWCR, TWEA);
                    setRegBit(TWCR, TWEN);
                    break;
                  }
                }
              }
            else                              {
              TWCR = 0; // i2c reset
              setRegBit(TWCR, TWEA);
              setRegBit(TWCR, TWEN);
              }
          }
      
        }

    
    if (i2csndaddr == false)         {return false; }
    if (slave->cfg.reg_size >= I2C_16B_REG) // if size >2, 16bit address is used
    {
        uint8_t dat = ((uint8_t) (reg_addr >> 8));
        bool i2cwr8 = false;
        TWDR = dat;
      
        i2c.start_wait = (uint16_t) millis();
      
        TWCR = (1 << TWINT) | (1 << TWEN);
      
        while (!(TWCR & (1 << TWINT))) { 
          if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
            TWCR = 0; // i2c reset
            setRegBit(TWCR, TWEA);
            setRegBit(TWCR, TWEN);
            i2cwr8 = false;
            break;
          }
        }
      
        if (TWI_STATUS == MT_DATA_ACK)    { i2cwr8 = true; }
      
        if (TWI_STATUS == MT_DATA_NACK){
          i2c.start_wait = (uint16_t) millis();
        
          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
        
          while ((TWCR & (1 << TWSTO))) {
            if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
              TWCR = 0; // i2c reset
              setRegBit(TWCR, TWEA);
              setRegBit(TWCR, TWEN);
              break;
            }
          }
        } else {
          TWCR = 0; // i2c reset
          setRegBit(TWCR, TWEA);
          setRegBit(TWCR, TWEN);
        }
      if (i2cwr8 == false)    {  return false; }
    }

        uint8_t dat = ((uint8_t) reg_addr);
        bool i2cwr8 = false;
        TWDR = dat;
      
        i2c.start_wait = (uint16_t) millis();
      
        TWCR = (1 << TWINT) | (1 << TWEN);
      
        while (!(TWCR & (1 << TWINT))) { 
          if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
            TWCR = 0; // i2c reset
            setRegBit(TWCR, TWEA);
            setRegBit(TWCR, TWEN);
            i2cwr8 = false;
            break;
          }
        }
      
        if (TWI_STATUS == MT_DATA_ACK)    { i2cwr8 = true; }
      
        if (TWI_STATUS == MT_DATA_NACK){
          i2c.start_wait = (uint16_t) millis();
        
          TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
        
          while ((TWCR & (1 << TWSTO))) {
            if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
              TWCR = 0; // i2c reset
              setRegBit(TWCR, TWEA);
              setRegBit(TWCR, TWEN);
              break;
            }
          }
        } else {
          TWCR = 0; // i2c reset
          setRegBit(TWCR, TWEA);
          setRegBit(TWCR, TWEN);
        }
    if (i2cwr8 == false)         {return false; }
  }

  bool i2cStart = false;
  i2c.start_wait = (uint16_t) millis();

  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

  while (!(TWCR & (1 << TWINT))) { 
    if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
        TWCR = 0; // i2c reset
        setRegBit(TWCR, TWEA);
        setRegBit(TWCR, TWEN);
        i2cStart = false; 
        break;
      }
  }

  if ((TWI_STATUS == START) || (TWI_STATUS == REPEATED_START))  { i2cStart = true; }
  if (TWI_STATUS == LOST_ARBTRTN){
    TWCR = 0; // i2c reset
    setRegBit(TWCR, TWEA);
    setRegBit(TWCR, TWEN);
  }
    
  if (i2cStart == false)                   { return false; }

        I2C_RW rw = I2C_READ;
        bool i2csndaddr = false;
        bool i2csndaddrset = false;
        TWDR = (slave->cfg.addr << 1) | rw;
      
        i2c.start_wait = (uint16_t) millis();
      
        TWCR = (1 << TWINT) | (1 << TWEN);
      
        while (!(TWCR & (1 << TWINT)))
        { if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout))  {
          TWCR = 0; // i2c reset
          setRegBit(TWCR, TWEA);
          setRegBit(TWCR, TWEN);
          i2csndaddr = false; i2csndaddrset = true; break; } }
      
        if(!i2csndaddrset) {
      
          if ((TWI_STATUS == MT_SLA_ACK) || (TWI_STATUS == MR_SLA_ACK)) { i2csndaddr = true; i2csndaddrset = true; }
      
          if(!i2csndaddrset) {
      
            if ((TWI_STATUS == MT_SLA_NACK) || (TWI_STATUS == MR_SLA_NACK)) {
                i2c.start_wait = (uint16_t) millis();
          
                TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
          
                while ((TWCR & (1 << TWSTO))) {
                  if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
                    TWCR = 0; // i2c reset
                    setRegBit(TWCR, TWEA);
                    setRegBit(TWCR, TWEN);
                    break;
                  }
                }
              }
            else                              {
              TWCR = 0; // i2c reset
              setRegBit(TWCR, TWEA);
              setRegBit(TWCR, TWEN);
              }
          }
      
        }
  
  if (i2csndaddr == false)            { return false; }

  for (uint16_t cnt = 0; cnt < bytes; cnt++)
  {
    uint8_t i2crd8 = true;
    bool ack = ((cnt == (bytes - 1)) ? false : true);

    i2c.start_wait = (uint16_t) millis();
    if (ack)  { TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); }
    else    { TWCR = (1 << TWINT) | (1 << TWEN); }
    while (!(TWCR & (1 << TWINT))) {
      if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)) {
        TWCR = 0; // i2c reset
        setRegBit(TWCR, TWEA);
        setRegBit(TWCR, TWEN);
        i2crd8 = false;
        break;
      }
    }
  
    if (TWI_STATUS == LOST_ARBTRTN)   {
      TWCR = 0; // i2c reset
      setRegBit(TWCR, TWEA);
      setRegBit(TWCR, TWEN);
      i2crd8 = false;
    }
    if(i2crd8 != false) {
      i2crd8 = ((((TWI_STATUS == MR_DATA_NACK) && (!ack)) || ((TWI_STATUS == MR_DATA_ACK) && (ack))) ? true : false);
    }

    if (i2crd8 == false)  { return false; }
    *data++ = TWDR;
    slave->reg_addr++;
  }

  bool i2cStop = true;
  i2c.start_wait = (uint16_t) millis();

  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);

  while ((TWCR & (1 << TWSTO))) {
    if ((((uint16_t) millis() - i2c.start_wait) >= i2c.cfg.timeout)){
      TWCR = 0; // i2c reset
      setRegBit(TWCR, TWEA);
      setRegBit(TWCR, TWEN);
      i2cStop =  false;
      break;
    }
  }

  if (i2cStop == false)                    { return false; }
  return true;
}


