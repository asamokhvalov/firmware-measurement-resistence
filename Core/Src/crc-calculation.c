#include "crc-calculation.h"

uint8_t calculate_crc_8 (uint8_t * array, uint8_t st_index, uint8_t lenght, uint8_t polynom, uint8_t init_value) {
	uint8_t crc_value = init_value;

	for (uint8_t i = st_index; i < st_index + lenght; i++) {
		crc_value ^= array[i];
		
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc_value & 0x80) != 0) {
				crc_value = (crc_value << 1) ^ polynom;
			}
			else {
				crc_value = crc_value << 1;
			}
		}
	}

	return crc_value;
}

uint16_t calculate_crc_16 (uint8_t * array, uint8_t st_index, uint8_t lenght, uint16_t polynom, uint16_t init_value) {
	uint16_t crc_value = init_value;

	for (uint8_t i = st_index; i < st_index + lenght; i++) {
		crc_value = crc_value ^ (array[i] << 8);
		
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc_value & 0x8000) != 0) {
				crc_value = (crc_value << 1) ^ polynom;
			}
			else {
				crc_value = crc_value << 1;
			}
		}
	}

	return crc_value;
}
