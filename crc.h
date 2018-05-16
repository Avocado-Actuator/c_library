#ifndef CRC_H_
#define CRC_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

static const uint8_t crc8_table[256];
unsigned crc8(unsigned, unsigned char const *, size_t);
unsigned crc8_slow(unsigned, unsigned char const *, size_t);

#endif /* CRC_H_ */
