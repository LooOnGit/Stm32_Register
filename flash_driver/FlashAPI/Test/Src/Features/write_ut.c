/*
 * write_ut.c
 *
 *  Created on: 7 Oct 2019
 *      Author: Arion
 */

#include "flash.h"
#include "flash_tests.h"
#include "test_utils.h"
#include "write_ut.h"


bool test_write() {
	uint8_t buffer[INVASIVE_TEST_BUFFER_SIZE];

	random_buffer(buffer, INVASIVE_TEST_BUFFER_SIZE);

	uint32_t write_crc = 0;
	uint32_t read_crc = 0;

	crc32(buffer, INVASIVE_TEST_BUFFER_SIZE, &write_crc);

	flash_erase_subsector(TEST_INVASIVE_ADDR);

	flash_write(TEST_INVASIVE_ADDR, buffer, INVASIVE_TEST_BUFFER_SIZE);
	flash_read(TEST_INVASIVE_ADDR, buffer, INVASIVE_TEST_BUFFER_SIZE);

	crc32(buffer, INVASIVE_TEST_BUFFER_SIZE, &read_crc);

	return write_crc == read_crc;
}
