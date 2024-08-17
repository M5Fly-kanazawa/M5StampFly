#ifndef SBUS_HPP
#define SBUS_HPP

#include <stdint.h>

void sbus_init(void);
uint8_t sbus_loop(void);
uint16_t sbus_getChannel(uint8_t ch);


#define SBUS_BAUDRATE (100000)
#endif