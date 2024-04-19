/*
 * erase_ut.c
 *
 *  Created on: 7 Oct 2019
 *      Author: Arion
 */


#include "flash_tests.h"
#include "flash.h"
#include "test_utils.h"
#include "erase_ut.h"


#define BEGIN_ADDR TEST_INVASIVE_ADDR
#define END_ADDR TEST_INVASIVE_ADDR + 0xF00 // Be careful: nothing happens if you try to write beyond this address

/*
 * In NOR flash memories, erased bits equal 1.
 */
bool __check_erased(uint8_t* buffer, uint32_t length) {
	for(uint32_t i = 0; i < length; i++) {
		if(buffer[i] != 0xFF) {
			return false;
		}
	}

	return true;
}

bool test_erase() {

	uint8_t buffer[INVASIVE_TEST_BUFFER_SIZE];


	/*
	 * Erase the sub-sector and check if it has really been erased.
	 */
	flash_erase_subsector(BEGIN_ADDR);

	flash_read(BEGIN_ADDR, buffer, INVASIVE_TEST_BUFFER_SIZE);

	if(!__check_erased(buffer, INVASIVE_TEST_BUFFER_SIZE)) {
		return false;
	}

	flash_read(END_ADDR - INVASIVE_TEST_BUFFER_SIZE, buffer, INVASIVE_TEST_BUFFER_SIZE);

	if(!__check_erased(buffer, INVASIVE_TEST_BUFFER_SIZE)) {
		return false;
	}


	/*
	 * Write some random data and check if it has really been written.
	 */

	random_buffer(buffer, INVASIVE_TEST_BUFFER_SIZE);

	flash_write(BEGIN_ADDR, buffer, INVASIVE_TEST_BUFFER_SIZE);

	flash_write(END_ADDR - INVASIVE_TEST_BUFFER_SIZE, buffer, INVASIVE_TEST_BUFFER_SIZE);

	flash_read(BEGIN_ADDR, buffer, INVASIVE_TEST_BUFFER_SIZE);

	if(__check_erased(buffer, INVASIVE_TEST_BUFFER_SIZE)) {
		return false;
	}

	flash_read(END_ADDR - INVASIVE_TEST_BUFFER_SIZE, buffer, INVASIVE_TEST_BUFFER_SIZE);


	if(__check_erased(buffer, INVASIVE_TEST_BUFFER_SIZE)) {
		return false;
	}


	/*
	 * Erase the sub-sector and check if it has really been erased.
	 */

	flash_erase_subsector(END_ADDR);

	flash_read(BEGIN_ADDR, buffer, INVASIVE_TEST_BUFFER_SIZE);
	if(!__check_erased(buffer, INVASIVE_TEST_BUFFER_SIZE)) {
		return false;
	}

	flash_read(END_ADDR - INVASIVE_TEST_BUFFER_SIZE, buffer, INVASIVE_TEST_BUFFER_SIZE);
	if(!__check_erased(buffer, INVASIVE_TEST_BUFFER_SIZE)) {
		return false;
	}


	return true;
}
