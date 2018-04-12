#ifndef _MYI2V_H
#define _MYI2C_H

#include <stdint.h>
#include <stdio.h>
#include <util/twi.h>

/* 400kHz */
#define I2C_FREQ 400000

#define I2C_TXN_DONE _BV(0)
#define I2C_TXN_ERR  _BV(1)

typedef struct i2c_op i2c_op_t;
typedef struct i2c_txn i2c_txn_t;

struct i2c_op {
  uint8_t address;
  uint8_t buflen;
  uint8_t bufpos;
  uint8_t *buf;
};

struct i2c_txn {
  struct i2c_txn *next;
  volatile uint8_t flags;
  uint8_t opslen;
  uint8_t opspos;
  struct i2c_op ops[];
};

static inline void i2c_op_init(i2c_op_t *o, uint8_t address, uint8_t *buf, uint8_t buflen) {
  o->address = address;
  o->buflen = buflen;
  o->bufpos = 0;
  o->buf = buf;
}

static inline void i2c_op_init_rd(i2c_op_t *o, uint8_t address, uint8_t *buf, uint8_t buflen) {
  i2c_op_init(o, (address << 1) | TW_READ, buf, buflen);
}

static inline void i2c_op_init_wr(i2c_op_t *o, uint8_t address, uint8_t *buf, uint8_t buflen) {
  i2c_op_init(o, (address << 1) | TW_WRITE, buf, buflen);
}

static inline void i2c_txn_init(i2c_txn_t *t, uint8_t opslen) {
  t->flags = 0;
  t->opslen = opslen;
  t->opspos = 0;
  t->next = NULL;
}

void i2c_init(void);
void i2c_post(i2c_txn_t *t);

#endif
