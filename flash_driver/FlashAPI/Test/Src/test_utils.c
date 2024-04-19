/*
 * test_utils.c
 *
 *  Created on: 8 Oct 2019
 *      Author: Arion
 */


#include "test_utils.h"



/* Simple public domain implementation of the standard CRC32 checksum.
 * Outputs the checksum for each file given as a command line argument.
 * Invalid file names and files that cause errors are silently skipped.
 * The program reads from stdin if it is called with no arguments. */

uint32_t crc32_for_byte(uint32_t r) {
  for(int j = 0; j < 8; ++j)
    r = (r & 1? 0: (uint32_t)0xEDB88320L) ^ r >> 1;
  return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, uint32_t length, uint32_t* crc) {
  static uint32_t table[0x100];
  if(!*table)
    for(uint32_t i = 0; i < 0x100; ++i)
      table[i] = crc32_for_byte(i);
  for(uint32_t i = 0; i < length; ++i)
    *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
}


uint64_t generator = 1ULL;

uint8_t random_byte() {
	generator = (17971 * generator + 16661) & 0xFF;
	return (uint8_t) generator;
}

void random_buffer(uint8_t* buffer, uint32_t length) {
	for(uint32_t i = 0; i < length; i++) {
		buffer[i] = random_byte();
	}
}
