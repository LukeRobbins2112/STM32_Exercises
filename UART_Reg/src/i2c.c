#include "i2c.h"
#include "stm32f103xb.h"

#define OWN_ADDR 0xAC // 0b10101100 -- random, but ensure lowest bit is 0 to match OAR register

// -------------------------------------------------
// Main Functionality
// -------------------------------------------------

uint8_t i2c_init(I2C_Init_t* i2c_init) {
	// ------------ GPIO setup ------------
	// PB6 as SCL, PB7 as SDA
	RCC->APB2ENR |= (0b1 << 3); // Enable clock for GPIOB

	// Alternate function output, open-drain
	GPIOB->CRL |= (0b1101 << (6 * 4)); // PB6
	GPIOB->CRL |= (0b1101 << (7 * 4)); // PB7

	// --------- Enable clock for I2C Peripheral ---------
	// Bit 21 --> I2C1 enable
	// Bit 22 --> I2C2 enable
	RCC->APB1ENR |= (0b1 << 21);

	// ------------ Clocking setup ------------

	// Tell the I2C about the incoming peripheral clock (PCLK) frequency
	// In our case, 32MHz
	I2C1->CR2 |= (0b100000 << 0);

	// Master Clock Config
	// CCR = 160 for 32 Mhz PCLK
	I2C1->CCR |= (0xA0 << 0);

	// Trise -- rise time for bus lines (via pullup resistors)
	// I think this may only be relevant for clock stretching
	I2C1->TRISE = 33; // (PCLK/1 million) + 1

	// ------------ General setup ------------

	// Slave Mode Setup
	if (i2c_init->role == I2C_SLAVE) {
		// Set own address -- pick something random
		I2C1->OAR1 = (OWN_ADDR << 0);

		// General call enabled
		I2C1->CR1 |= (0b1 << 6);

		// By default, we clock stretch
		if (i2c_init->clk_stretch == NO_STRETCH) {
			I2C1->CR1 |= (0b1 << 7);
		}

		// Ready to send initial ack
		I2C1->CR1 |= (0b1 << 10);
	}

	// Enable the peripheral, once config is done
	I2C1->CR1 |= (0b1 << 0);

	return 0;
}

uint8_t i2c_send_data(uint8_t data) {
	return 0;
}

uint8_t i2c_receive_data() {
	return 0;
}
