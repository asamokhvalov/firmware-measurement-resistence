#include "dev-info.h"
#include "gpio.h"

dev_info dev_r;

void dev_init (dev_info * _dev_info)
{
	_dev_info->status_dev = STAGE_MEASUREMENT;

//	_dev_info->gpio_mul = MULTIPLE_X_1;
//	_dev_info->gpio_div = DIVIDER_47_M;

	_dev_info->gpio_mul = 0;
	_dev_info->gpio_div = 0;
	
	_dev_info->adc_data_meas = 0;
	_dev_info->adc_data_ref_volt = 0;

	_dev_info->thermocouple_1 = 0;
	_dev_info->thermocouple_2 = 0;
	
	_dev_info->cur_adc_data_meas = 0;
	_dev_info->cur_adc_data_ref_volt = 0;

	for (uint8_t i = 0; i < CAPTURE_DATA_SIZE; i++)
	{
		_dev_info->capture_adc_data_meas[i] = 0;
		_dev_info->capture_adc_data_ref_volt[i] = 0;
	}

//	Write_MUL (MULTIPLE_X_1, GPIO_PIN_SET);
//	Write_DIV (DIVIDER_47_M, GPIO_PIN_SET);
}

void dev_processing (dev_info * _dev_info) {
	uint8_t stage = _dev_info->status_dev;

	switch (stage) {
		case STAGE_MEASUREMENT: {
			break;
		}
		case STAGE_CALCULATION: {
			break;
		}
		case STAGE_CHANGE_DIV_MUL: {
			break;
		}
		case STAGE_SENDING: {
			break;
		}
		default: {
			break;
		}
	}
}

uint16_t get_sys_time (dev_info * _dev_info)
{
	return _dev_info->sys_time;
}

void set_sys_time (dev_info * _dev_info, uint16_t value)
{
	_dev_info->sys_time = value;
}

uint8_t get_status_dev (dev_info * _dev_info) {
	return _dev_info->status_dev;
}

void set_status_dev (dev_info * _dev_info, uint8_t value) {
	_dev_info->status_dev = value;
}

uint8_t get_flag_proc_data (dev_info * _dev_info) {
	return _dev_info->flag_proc_data;
}

void set_flag_proc_data (dev_info * _dev_info, uint8_t value) {
	_dev_info->flag_proc_data = value;
}

uint8_t get_r_multiple (dev_info * _dev_info) {
	return _dev_info->gpio_mul;
}

void set_r_multiple (dev_info * _dev_info, uint8_t value) {
	_dev_info->gpio_mul = value;
}

uint8_t get_r_divide (dev_info * _dev_info) {
	return _dev_info->gpio_div;
}

void set_r_divide (dev_info * _dev_info, uint8_t value) {
	_dev_info->gpio_div = value;
}

uint16_t get_adc_data_meas (dev_info * _dev_info) {
	return _dev_info->adc_data_meas;
}

void set_adc_data_meas (dev_info * _dev_info, uint16_t value) {
	_dev_info->adc_data_meas = value;
}

uint16_t get_adc_data_ref_volt (dev_info * _dev_info) {
	return _dev_info->adc_data_ref_volt;
}

void set_adc_data_ref_volt (dev_info * _dev_info, uint16_t value) {
	_dev_info->adc_data_ref_volt = value;
}

uint16_t get_cur_adc_data_meas (dev_info * _dev_info) {
	return _dev_info->cur_adc_data_meas;
}

void set_cur_adc_data_meas (dev_info * _dev_info, uint16_t value) {
	_dev_info->cur_adc_data_meas = value;
}

uint16_t get_cur_adc_data_ref_volt (dev_info * _dev_info) {
	return _dev_info->cur_adc_data_ref_volt;
}

void set_cur_adc_data_ref_volt (dev_info * _dev_info, uint16_t value) {
	_dev_info->cur_adc_data_ref_volt = value;
}

uint16_t get_thermocouple_1 (dev_info * _dev_info) {
	return _dev_info->thermocouple_1;
}

void set_thermocouple_1 (dev_info * _dev_info, uint16_t value) {
	_dev_info->thermocouple_1 = value;
}

uint16_t get_thermocouple_2 (dev_info * _dev_info) {
	return _dev_info->thermocouple_2;
}

void set_thermocouple_2 (dev_info * _dev_info, uint16_t value) {
	_dev_info->thermocouple_2 = value;
}

uint16_t get_capture_cur_adc_data_meas (dev_info * _dev_info, uint8_t index) {
	return _dev_info->capture_adc_data_meas[index];
}

void set_capture_cur_adc_data_meas (dev_info * _dev_info, uint16_t value, uint8_t index) {
	_dev_info->capture_adc_data_meas[index] = value;
}

uint16_t get_capture_cur_adc_data_ref_volt (dev_info * _dev_info, uint8_t index) {
	return _dev_info->capture_adc_data_ref_volt[index];
}

void set_capture_cur_adc_data_ref_volt (dev_info * _dev_info, uint16_t value, uint8_t index) {
	_dev_info->capture_adc_data_ref_volt[index] = value;
}
