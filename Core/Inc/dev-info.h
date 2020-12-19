#ifndef DEV_INGO_H
#define DEV_INGO_H

#include "stm32f3xx.h"

#define MAX_SYS_TICK_MS (uint16_t)65000

typedef enum {
	START_VALUE_SIGNAL_MIN = 500,	// 200 * 0.000805 = 0.1611 V
	START_VALUE_SIGNAL_MAX = 3900	// 3725 * 0.000805 = 3.001098 V
}
CAPTURE_VALUE_BORDERS;

typedef enum {
	PERIOD_SEND_DELAY = 1000,
	PERIOD_CHANGE_RESISTOR = 100
}
PERIOD_TIMING;

/* ---------------------------------------- */

typedef enum {
	STAGE_MEASUREMENT = 0x00,
	STAGE_CALCULATION = 0x01,
	STAGE_SENDING = 0x02,
	STAGE_CHANGE_DIV_MUL = 0x03
}
STATUS_DEVICE;

typedef enum {
	MULTIPLE_X_1 = 0,	// NC PIN
	MULTIPLE_X_2 = 1,	// PA 5 -> A4
	MULTIPLE_X_5 = 2,	// PA 6 -> A5
	MULTIPLE_X_10 = 3, // PA 7 -> A6
	MULTIPLE_X_100 = 4, // PA 8 -> D9
	MULTIPLE_X_1000 = 5 // PA 11 -> D10
}
R_MULTIPLE;

typedef enum {
	DIVIDER_10_K = 0, // PA 4 -> A3
	DIVIDER_100_K = 1, // PA 3 -> A2
	DIVIDER_1_M = 2, // PA 2 -> A7
	DIVIDER_10_M = 3, // PA 1 -> A1
	DIVIDER_47_M = 4 // PA 0 -> A0
}
R_DIVIDE;

typedef enum {
	CAPTURE_DATA_SIZE = 50,
//	CAPTURE_DATA_MEAS_SIZE  = 50,	// measurement voltage
//	CAPTURE_RISING_REF_VOLT_SIGNAL  = 50	// reference voltage
}
ADC_ARRAY_DATA_SIZE;

typedef enum {
	PROCESS_DATA_FREE = 0,
	PROCESS_DATA_BUSY = 1
}
FLAG_PROCESS_DATA;

typedef struct __attribute__((packed)) _dev_info {
	volatile uint8_t status_dev;
	
	volatile uint16_t sys_time;

	volatile uint8_t flag_proc_data;
	
	volatile uint8_t gpio_mul;	// current stage of multiple resistors
	volatile uint8_t gpio_div;	// current stage of divide resistors

	volatile uint16_t adc_data_meas;			// average data from array to send
	volatile uint16_t adc_data_ref_volt;	// average data from array to send
	
	volatile uint16_t cur_adc_data_meas;		// captured data in IT
	volatile uint16_t cur_adc_data_ref_volt; 	// captured data in IT

	volatile uint16_t thermocouple_1;		// 
	volatile uint16_t thermocouple_2;		//
	
	volatile uint16_t capture_adc_data_meas[CAPTURE_DATA_SIZE];
	volatile uint16_t capture_adc_data_ref_volt[CAPTURE_DATA_SIZE];
}
dev_info;

extern dev_info dev_r;

void dev_init (dev_info * _dev_info);

void dev_processing (dev_info * _dev_info);

uint16_t get_sys_time (dev_info * _dev_info);
void set_sys_time (dev_info * _dev_info, uint16_t value);

uint8_t get_status_dev (dev_info * _dev_info);
void set_status_dev (dev_info * _dev_info, uint8_t value);

uint8_t get_flag_proc_data (dev_info * _dev_info);
void set_flag_proc_data (dev_info * _dev_info, uint8_t value);

uint8_t get_r_multiple (dev_info * _dev_info);
void set_r_multiple (dev_info * _dev_info, uint8_t value);

uint8_t get_r_divide (dev_info * _dev_info);
void set_r_divide (dev_info * _dev_info, uint8_t value);

uint16_t get_adc_data_meas (dev_info * _dev_info);
void set_adc_data_meas (dev_info * _dev_info, uint16_t value);

uint16_t get_adc_data_ref_volt (dev_info * _dev_info);
void set_adc_data_ref_volt (dev_info * _dev_info, uint16_t value);

uint16_t get_cur_adc_data_meas (dev_info * _dev_info);
void set_cur_adc_data_meas (dev_info * _dev_info, uint16_t value);

uint16_t get_cur_adc_data_ref_volt (dev_info * _dev_info);
void set_cur_adc_data_ref_volt (dev_info * _dev_info, uint16_t value);

uint16_t get_thermocouple_1 (dev_info * _dev_info);
void set_thermocouple_1 (dev_info * _dev_info, uint16_t value);

uint16_t get_thermocouple_2 (dev_info * _dev_info);
void set_thermocouple_2 (dev_info * _dev_info, uint16_t value);

uint16_t get_capture_cur_adc_data_meas (dev_info * _dev_info, uint8_t index);
void set_capture_cur_adc_data_meas (dev_info * _dev_info, uint16_t value, uint8_t index);

uint16_t get_capture_cur_adc_data_ref_volt (dev_info * _dev_info, uint8_t index);
void set_capture_cur_adc_data_ref_volt (dev_info * _dev_info, uint16_t value, uint8_t index);

#endif
