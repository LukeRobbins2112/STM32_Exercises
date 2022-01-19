#ifndef _RTC_H_
#define _RTC_H_

#include <stdint.h>

#define DS3231_I2C_ADDR 0x68 // I2C Address

uint8_t rtc_read_reg(uint8_t reg);
uint8_t rtc_burst_read(uint8_t reg, uint8_t* data, uint8_t len);
uint8_t rtc_set_time(uint8_t hour, uint8_t min, uint8_t sec);
uint8_t rtc_get_time(uint8_t* hour, uint8_t* min, uint8_t* sec);


#endif // _RTC_H_
