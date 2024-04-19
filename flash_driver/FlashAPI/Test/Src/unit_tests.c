/*
 * unit_tests.c
 *
 *  Created on: 1 Oct 2019
 *      Author: Arion
 */


#include "flash_tests.h"
#include "read_ut.h"
#include "write_ut.h"
#include "erase_ut.h"

test_result test_features(uint8_t features) {
	test_result result;

	result.status = TEST_RUNNING;
	result.score = 0b11110001;

	if(features & FEATURE_READ) {
		if(test_read()) {
			result.score |= FEATURE_READ;
		}
	}

	if(features & FEATURE_WRITE) {
		if(test_write()) {
			result.score |= FEATURE_WRITE;
		}
	}

	if(features & FEATURE_ERASE) {
		if(test_erase()) {
			result.score |= FEATURE_ERASE;
		}
	}

	result.status = TEST_FINISHED;

	return result;
}

uint8_t is_test_result_satisfying(test_result result, uint64_t standard) {
	uint8_t matching_features = (result.score & standard);

	return result.status == TEST_FINISHED && matching_features == standard;
}
