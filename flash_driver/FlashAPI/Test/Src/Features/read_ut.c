/*
 * read_ut.c
 *
 *  Created on: 7 Oct 2019
 *      Author: Arion
 */

#include "read_ut.h"
#include "flash.h"
#include "flash_tests.h"
#include "test_utils.h"


#define NUM_SHIFT_TESTS 32

#define WRITE_ONCE_CRC_RESULT 0x8906affc


void __write_once() {
	uint8_t write_buffer[GENTLE_TEST_BUFFER_SIZE];

	random_buffer(write_buffer, GENTLE_TEST_BUFFER_SIZE);

	uint32_t write_crc = 0;

	crc32(write_buffer, GENTLE_TEST_BUFFER_SIZE, &write_crc);

	flash_erase_subsector(TEST_GENTLE_ADDR);
	flash_write(TEST_GENTLE_ADDR, write_buffer, GENTLE_TEST_BUFFER_SIZE);

}

/*
 * Compares the two given buffers.
 * The source is shifted by the specified offset and its elements are compared one by one with the source buffer.
 * This function returns true if and only if all elements (shifted) are equal.
 */
bool __verify(uint8_t* source, uint8_t* buffer, uint32_t offset, uint32_t length) {
	for(uint32_t i = 0; i < length; i++) {
		if(source[i + offset] != buffer[i]) {
			return false;
		}
	}

	return true;
}


/*
 * Sub-tests start
 */

bool __verify_source_buffer(uint8_t* source) {
	uint32_t crc = 0;
	crc32(source, GENTLE_TEST_BUFFER_SIZE, &crc);
	return crc == WRITE_ONCE_CRC_RESULT;
}

bool __pass_shift_tests(uint8_t* source) {
	uint8_t buffer[GENTLE_TEST_BUFFER_SIZE];

	for(uint32_t i = 0; i < NUM_SHIFT_TESTS; i++) {
		uint8_t shift_amount = random_byte();
		uint8_t length = random_byte() % (GENTLE_TEST_BUFFER_SIZE - shift_amount);

		flash_read(TEST_GENTLE_ADDR + shift_amount, buffer, length);

		if(!__verify(source, buffer, shift_amount, length)) {
			return false;
		}
	}

	return true;
}

/*
 * Sub-tests end
 */

bool test_read() {
	// __write_once();

	uint8_t buffer[GENTLE_TEST_BUFFER_SIZE];

	flash_read(TEST_GENTLE_ADDR, buffer, GENTLE_TEST_BUFFER_SIZE);


	if(!__verify_source_buffer(buffer)) {
		return false;
	}

	if(!__pass_shift_tests(buffer)) {
		return false;
	}

	return true;
}
