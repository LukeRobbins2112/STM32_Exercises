#ifndef _I2C_H_
#define _I2C_H_

#include <stdint.h>

typedef enum I2C_Role {
	I2C_SLAVE = 0,
	I2C_MASTER = 1,
} I2C_Role_e;

typedef enum Clock_Stretch {
	STRETCH = 0,
	NO_STRETCH = 1,
} Clock_Stretch_e;

typedef struct I2C_Init {
	I2C_Role_e role;
	Clock_Stretch_e clk_stretch;
} I2C_Init_t;

typedef enum I2C_RW {
	I2C_WRITE = 0,
	I2C_READ = 1,
} I2C_RW_e;

typedef enum I2C_Ack {
	I2C_ACK = 0,
	I2C_NO_ACK = 1,
} I2C_Ack_e;

uint8_t i2c_init(I2C_Init_t* i2c_init);
uint8_t i2c_start_transaction();
uint8_t i2c_end_transaction();
uint8_t i2c_send_addr(uint8_t addr, I2C_RW_e rw);
uint8_t i2c_send_data(uint8_t data);
uint8_t i2c_receive_data(I2C_Ack_e ack);

void wait_for_ready();
void send_start();
void send_stop();

// Full transactions
uint8_t i2c_transaction(uint8_t addr, I2C_RW_e rw, uint8_t* data, uint16_t len);


#endif // _I2C_H_
