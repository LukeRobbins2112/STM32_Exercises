#include "i2c.h"
#include "stm32f103xb.h"

#define OWN_ADDR 0xAC // 0b10101100 -- random, but ensure lowest bit is 0 to match OAR register

// Flags
#define BUS_BUSY 1
#define I2C_RXNE 6
#define I2C_TXE 7
#define ARLO 9

#define I2C_ENABLE 0
#define I2C_START 8
#define I2C_STOP  9
#define SWRST 15

static uint32_t arlo_count = 0;

// -------------------------------------------------
// Function Declarations
// -------------------------------------------------
static void I2C_ClearBusyFlagErratum();

// -------------------------------------------------
// Main Functionality
// -------------------------------------------------

uint8_t i2c_init(I2C_Init_t* i2c_init) {


	// --------- Enable clock for I2C Peripheral & GPIOs ---------
	// Bit 21 --> I2C1 enable
	// Bit 22 --> I2C2 enable
	RCC->APB1ENR |= (0b1 << 21); // I2C1
	RCC->APB2ENR |= (0b1 << 3); // Enable clock for GPIOB

	if (I2C1->SR2 & 0b1 << BUS_BUSY) {
		I2C_ClearBusyFlagErratum();
	} else {
		// ------------ Normal GPIO setup ------------
		// PB6 as SCL, PB7 as SDA
		// Alternate function output, open-drain
		GPIOB->CRL |= (0b1101 << (6 * 4)); // PB6
		GPIOB->CRL |= (0b1101 << (7 * 4)); // PB7
	}

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

uint8_t i2c_transaction(uint8_t addr, I2C_RW_e rw, uint8_t* data, uint16_t len) {
	// Take bus control, send start
	i2c_start_transaction();

	// Send address with R/W bit
	i2c_send_addr(addr, rw);

	for (int i = 0; i < len; i++) {
		if (rw == I2C_WRITE) {
			i2c_send_data(data[i]);
		} else {
			data[i] = i2c_receive_data(I2C_ACK);
		}
	}

	// Send stop condition, wait to become slave
	i2c_end_transaction();

	return 0;
}

uint8_t i2c_start_transaction() {
	// Take control of the bus, send start signal
	wait_for_ready();
	send_start();
	return 0;
}

uint8_t i2c_end_transaction() {
	send_stop();
	return 0;
}

uint8_t i2c_send_addr(uint8_t addr, I2C_RW_e rw) {
	// Shift address to bits 1-7, set bit 0 for r/w
	uint8_t prep_addr = addr << 1;
	prep_addr |= (rw << 0);

	// Send slave address, wait for confirmation
	I2C1->DR = prep_addr;
	while (0 == (I2C1->SR1 & (0b1 << 1))) {
		if (0 != (I2C1->SR1 & (0b1 << ARLO))) {
			// Lost arbitration -- try again.
			if (++arlo_count > 5) return 1;
			return i2c_send_addr(addr, rw);
		}
	}
	uint32_t stat = I2C1->SR2; // Read both SRs to clear the flag

	return 0;
}

uint8_t i2c_send_data(uint8_t data) {
	// Assumes we already started and sent addr

	// Put data in outgoing data reg, wait for confirmation of send
	I2C1->DR = data;
	while (0 == (I2C1->SR1 & (0b1 << I2C_TXE))) {}

	return 0;
}

uint8_t i2c_receive_data(I2C_Ack_e ack) {
	// Set ACK bit
	I2C1->CR1 |= (ack << 10);

	// Monitor RxNE flag. Once set, read data register
	while(0 == (I2C1->SR1 & (0b1 << I2C_RXNE))) {}

	uint8_t data = I2C1->DR;
	return data;
}

// I2C Utility functions
void wait_for_ready() {
	// Spin waiting for the bus to be available
	// @TODO maybe add a timeout
	while (0 != (I2C1->SR2 & (0b1 << BUS_BUSY))) {}
}

void send_start() {
	I2C1->CR1 |= (0b1 << I2C_START);

	// Wait for start condition flag to set
	// Cleared on SR1 read + DR write (which occurs on send_addr)
	while(0 == (I2C1->SR1 & (0b1 << 0))) {}
	int stat = I2C2->SR2; // Clear START flag
}


void send_stop() {
	I2C1->CR1 |= (0b1 << I2C_STOP);

	// Wait for M/S flag to indicate transition to slave mode
	while(0 != (I2C1->SR2 & (0b1 << 0))) {}
}


static void I2C_ClearBusyFlagErratum() {
	// 1. Disable the I2C peripheral by clearing the PE bit in I2Cx_CR1 register.
	I2C1->CR1 &= ~(0b1 << I2C_ENABLE);

	// 2. Configure SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	// Open Drain Mode: A “0” in the Output register activates the N-MOS while a “1” in
	// the Output register leaves the port in Hi-Z (the P-MOS is never activated)
	GPIOB->CRL |= (0b0101 << (6 * 4)); // PB6
	GPIOB->CRL |= (0b0101 << (7 * 4)); // PB7
	GPIOB->ODR |= (0b1 << 6);
	GPIOB->ODR |= (0b1 << 7);

	// 3. Check SCL and SDA High level in GPIOx_IDR.
	while (!(GPIOB->IDR & 0b1 << 6)) {}
	while (!(GPIOB->IDR & 0b1 << 7)) {}

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	// Already configured as GP OD, so just write Low level?
	GPIOB->ODR &= ~(0b1 << 7);

	// 5. Check SDA Low level in GPIOx_IDR.
	while ((GPIOB->IDR & 0b1 << 7)) {}

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	GPIOB->ODR &= ~(0b1 << 6);

	// 7. Check SCL Low level in GPIOx_IDR
	while ((GPIOB->IDR & 0b1 << 6)) {}

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIOB->ODR |= (0b1 << 6);

	// 9. Check SCL High level in GPIOx_IDR.
	while (!(GPIOB->IDR & 0b1 << 6)) {}

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR)
	GPIOB->ODR |= (0b1 << 7);

	// 11. Check SDA High level in GPIOx_IDR.
	while (!(GPIOB->IDR & 0b1 << 7)) {}

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIOB->CRL |= (0b1101 << (6 * 4)); // PB6
	GPIOB->CRL |= (0b1101 << (7 * 4)); // PB7

	// 13. Set SWRST bit in I2Cx_CR1 register.
	I2C1->CR1 |= (0b1 << SWRST);

	// 14. Clear SWRST bit in I2Cx_CR1 register.
	I2C1->CR1 &= ~(0b1 << SWRST);

	// 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register.
	// Do this after the rest of the configuration
	// I2C1->CR1 |= (0b1 << I2C_ENABLE);
}



