#include "rtc.h"
#include "i2c.h"

enum RTC_REGISTERS {
	SECS_RTC = 0x0,
	MINS_RTC = 0x1,
	HOURS_RTC = 0x2,
	DAYS_RTC = 0x3,
	DATE_RTC = 0x4,
	MONTHS_RTC = 0x5,
	YEARS_RTC = 0x6
};


// helper
uint8_t set_reg_ptr(uint8_t regNum) {
	i2c_transaction(DS3231_I2C_ADDR, I2C_WRITE, &regNum, 1);
	return 0;
}


// Main logic

uint8_t rtc_read_reg(uint8_t reg) {
	// Tell RTC chip where we want to do our read
	set_reg_ptr(reg);

	// Perform our read
	uint8_t regVal;
	i2c_transaction(DS3231_I2C_ADDR, I2C_READ, &regVal, 1);
	return regVal;
}

uint8_t rtc_burst_read(uint8_t reg, uint8_t* data, uint8_t len) {
	// Tell RTC chip where we want to do our read
	set_reg_ptr(reg);

	// Perform our read
	i2c_transaction(DS3231_I2C_ADDR, I2C_READ, &data, len);
	return 0;
}

uint8_t rtc_get_time(uint8_t* hour, uint8_t* min, uint8_t* sec) {
	// Set start for burst rest
	set_reg_ptr(SECS_RTC);

	// Perform our read
	uint8_t data[3];
	i2c_transaction(DS3231_I2C_ADDR, I2C_READ, &data, 3);
	*sec = data[0];
	*min = data[1];
	*hour = data[2];

	return 0;
}


