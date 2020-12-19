#include "d-protocol.h"

#include "dev-info.h"
#include "crc-calculation.h"

/* For HAL. */
#include "usart.h"

data_protocol data_resistance;

uint8_t get_flag_tc_dma (data_protocol * _data_protocol) {
	return _data_protocol->flag_tc_dma;
}

void set_flag_tc_dma (data_protocol * _data_protocol, uint8_t flag_value) {
	_data_protocol->flag_tc_dma = flag_value;
}

uint8_t get_cur_size_buf_in (data_protocol * _data_protocol) {
	return _data_protocol->cur_size_buf_in;
}

void set_cur_size_buf_in (data_protocol * _data_protocol, uint8_t value) {
	_data_protocol->cur_size_buf_in = value;
}

uint8_t get_cur_size_buf_out (data_protocol * _data_protocol) {
	return _data_protocol->cur_size_buf_out;
}

void set_cur_size_buf_out (data_protocol * _data_protocol, uint8_t value) {
	_data_protocol->cur_size_buf_out = value;
}

inline void search_message (data_protocol * _data_protocol) {
	/* Number of data part bytes. */
	uint16_t num_bytes = 0;

	/* Number of bytes in message. */
	uint8_t size_message = 0;

	/* Index byte in HW buffer for copy this to IN buffer. After copy HW byte clear by this index. */
	uint8_t index_hw_buf = 0;

	/* If message is offset. Number of bytes in no offset and offset parts. */
	uint8_t num_bytes_in_st_buf = 0;
	uint8_t num_bytes_to_end_buf = 0;

	/* Index of the last byte. */
	uint8_t index_end_message = 0;

	uint8_t start_byte = 0;
	uint8_t end_line_byte = 0;
	uint8_t carriage_return_byte = 0;
	
	for (uint8_t i = 0; i < BUF_HW_SIZE; i++) {
		start_byte = _data_protocol->buf_hw[i];
		
		if (start_byte == START_SYMBOL) {
			size_message++;
			
			/* Search numbers of data bytes in message. Control offset buffer. */
			if (BUF_HW_SIZE - i == 2) {
				num_bytes = (uint16_t)(_data_protocol->buf_hw[BUF_HW_SIZE - 1]);
			}
			else if (BUF_HW_SIZE - i == 1) {
				num_bytes = (uint16_t)(_data_protocol->buf_hw[0]);
			}
			else {
				num_bytes = (uint16_t)(_data_protocol->buf_hw[i + 1]);
			}
			
			/* Add byte of size message. */
			size_message++;

			/* Add number of bytes in message. */
			size_message += (uint8_t)num_bytes;
		
			/* Chech for offset. +1 for bytes consist of number of bytes in message and +1 for start byte. */
			size_message += CRC_8_SIZE + END_SYMBOL_SIZE;
			
			/* No shift message. */
			if (BUF_HW_SIZE - i >= size_message) {
				
				/* Index of the last byte message in HW buffer. */
				index_end_message = i + size_message;

				end_line_byte = _data_protocol->buf_hw[index_end_message - 2];
				carriage_return_byte = _data_protocol->buf_hw[index_end_message - 1];
				
				/* Check end of HEAP on END_PACKAGE_SYMBOL. */
				if ((end_line_byte == END_LINE) && (carriage_return_byte == CARRIAGE_RETURN)) {

					/* Get index start byte in HW buffer. */
					index_hw_buf = i;

					size_message = index_end_message - i + 1;
					
					/* Copy message from HW buffer to IN buffer. */
					for (uint8_t j = 0; j < size_message; j++) {

						/* Copy byte from HW buffer to IN buffer. */
						_data_protocol->buf_in[j] = _data_protocol->buf_hw[index_hw_buf];

						/* Clear HW buffer. */
						_data_protocol->buf_hw[index_hw_buf] = 0x00;

						index_hw_buf++;
					}
					
					set_cur_size_buf_in (_data_protocol, size_message);
					
					parse_message (_data_protocol);
				}
				else {
					/* Clear. */
//					for (uint8_t j = i; j < index_end_message + 1; j++) {
//						/* Clear HW buffer. */
//						_data_protocol->buf_hw[j] = 0x00;
//					}					
				}
			}
			/* Shift message. */
			else {
				/* Get number of bytes to the end of HW buffer. */
				num_bytes_to_end_buf = BUF_HW_SIZE - i;

				/* Index of the last byte message in HW buffer. */
				index_end_message = size_message - num_bytes_to_end_buf - 1;

				/* Check end of HEAP on END_PACKAGE_SYMBOL. */
				if (_data_protocol->buf_hw[index_end_message] == END_LINE) {

					/* Get index start byte the no offset part of message in HW buffer. */
					index_hw_buf = i;

					/* Copy the no offset part of message from HW buffer to IN buffer. */
					for (uint8_t j = 0; j < num_bytes_to_end_buf; j++) {

						/* Copy byte from HW buffer to IN buffer. */
						_data_protocol->buf_in[j] = _data_protocol->buf_hw[index_hw_buf];

						/* Clear HW buffer. */
						_data_protocol->buf_hw[index_hw_buf] = 0x00;

						index_hw_buf++;
					}

					/* Get index start byte the offset part of message in HW buffer. */
					index_hw_buf = 0;
					
					/* Get number of bytes from the offset of HW buffer. */
					num_bytes_in_st_buf = size_message - num_bytes_to_end_buf;

					/* Copy the offset part of message from HW buffer to IN buffer. */
					for (uint8_t j = num_bytes_to_end_buf; j < num_bytes_to_end_buf + num_bytes_in_st_buf; j++) {

						/* Copy byte from HW buffer to IN buffer. */
						_data_protocol->buf_in[j] = _data_protocol->buf_hw[index_hw_buf];

						/* Clear HW buffer. */
						_data_protocol->buf_hw[index_hw_buf] = 0x00;

						index_hw_buf++;
					}
					
//					parse_heap (_data_protocol);
				}
				else {
				}
			}
		}
	}	
}


/**
 * @brief  
 * @note   
 * @param  _data_protocol: 
 * @retval None
 */
inline static void parse_message (data_protocol * _data_protocol) {
	uint8_t calc_crc_8 = 0;
	uint8_t mess_crc_8 = 0;
	
	uint8_t command = 0;
	
	calc_crc_8 = calculate_crc_8 ((uint8_t *)_data_protocol->buf_in, 1, DATA_LENGHT, 0x0D, 0x00);
	mess_crc_8 = _data_protocol->buf_in[3];
	
	if (calc_crc_8 == mess_crc_8) {
		command = _data_protocol->buf_in[2];
		
		if (command == START_COMMAND) {
//			adc_start ();
//
//			adc_calibrate ();
//
//			set_status_dev (&dev_r, STAGE_CAPTURE_RISING);
//
//			tim17_enable ();
//			tim14_enable ();
			
		}
		else if (command == STOP_COMMAND) {
		}
		else {

		}
		
	}
	else {
		__ASM("NOP");
	}
}

void send_message (data_protocol * _data_protocol) {
	prepare_data_to_transfer ();
	
	send_data (_data_protocol);
	/* Put user send function if you do not use HAL. */
//	send_data_usart_dma ((uint8_t *)_data_protocol->buf_out, 9);
}

static void prepare_data_to_transfer (void) {
	static uint8_t r_div_val = 0;
	static uint8_t r_mul_val = 0;

	static uint16_t adc_data_meas_val = 0;
	static uint16_t adc_data_ref_volt_val = 0;

	static uint16_t thermocouple_1 = 0;
	static uint16_t thermocouple_2 = 0;
	
	static uint8_t count_buffer = 0;
	static uint8_t count_data = 0;
	
	static uint8_t crc_8 = 0;
	
	r_div_val = get_r_divide (&dev_r);
	r_mul_val = get_r_multiple (&dev_r);

	adc_data_meas_val = get_adc_data_meas (&dev_r);
	adc_data_ref_volt_val = get_adc_data_ref_volt (&dev_r);

	thermocouple_1 = get_thermocouple_1 (&dev_r);
	thermocouple_2 = get_thermocouple_2 (&dev_r);
	
	count_buffer = 0;
	count_data = 0;
	data_resistance.cur_size_buf_out = 0;

	data_resistance.buf_out[0] = START_SYMBOL;
	count_buffer++;
	
	/* Fill data part. */
	data_resistance.buf_out[2] = (uint8_t)((r_div_val << 4) | r_mul_val);
	count_data++;

	data_resistance.buf_out[3] = (uint8_t)((adc_data_meas_val & 0xFF00) >> 8);
	count_data++;
	data_resistance.buf_out[4] = (uint8_t)(adc_data_meas_val & 0x00FF);
	count_data++;
	
	data_resistance.buf_out[5] = (uint8_t)((adc_data_ref_volt_val & 0xFF00) >> 8);
	count_data++;
	data_resistance.buf_out[6] = (uint8_t)(adc_data_ref_volt_val & 0x00FF);
	count_data++;
	
	data_resistance.buf_out[7] = (uint8_t)((thermocouple_1 & 0xFF00) >> 8);
	count_data++;
	data_resistance.buf_out[8] = (uint8_t)(thermocouple_1 & 0x00FF);
	count_data++;
	
	data_resistance.buf_out[9] = (uint8_t)((thermocouple_2 & 0xFF00) >> 8);
	count_data++;
	data_resistance.buf_out[10] = (uint8_t)(thermocouple_2 & 0x00FF);
	count_data++;
	
	/* Fill number of transfer bytes. */
	data_resistance.buf_out[1] = count_data;
	count_buffer++;
	
	crc_8 = calculate_crc_8 ((uint8_t *)data_resistance.buf_out, 2, count_data, 0x0D, 0x00);
	data_resistance.buf_out[11] = crc_8;
	count_buffer++;
	
	data_resistance.buf_out[12] = END_LINE;
	count_buffer++;
	data_resistance.buf_out[13] = CARRIAGE_RETURN;
	count_buffer++;
	
	data_resistance.cur_size_buf_out = count_buffer + count_data;
}
