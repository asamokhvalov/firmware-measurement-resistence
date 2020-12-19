#ifndef D_PROTOCOL_H
#define D_PROTOCOL_H

#include "stm32f3xx.h"

typedef enum {
	START_COMMAND = 0x11,
	STOP_COMMAND = 0x22
}
COMMAND_CODE;

/* ---------------------------------------- */
typedef enum {
	START_SYMBOL = 0x3A,
	END_LINE = 0x0A,
	CARRIAGE_RETURN = 0x0D
}
PROTOCOL_BYTES;

/**/
typedef enum {
	BUF_HW_SIZE = 32,
	BUF_IN_SIZE = 32,
	BUF_OUT_SIZE = 32, 
	
	CRC_8_SIZE = 1,
	END_SYMBOL_SIZE = 2,
	
	DATA_LENGHT = 2,
}
DATA_SIZE;

/**/
typedef enum {
	DMA_TC_RESET = 64,
	DMA_TC_SET = 64,
}
DMA_TRANSFER_STATUS;

typedef struct __attribute__((packed)) _data_protocol {
	volatile uint8_t buf_hw[BUF_HW_SIZE];	/**	1. @brief Buffer for saving data from receive hardware line. */

	volatile uint8_t buf_in[BUF_IN_SIZE];	/**	2. @brief Buffer for correct messages searched  */

	volatile uint8_t buf_out[BUF_OUT_SIZE];	/**	3. @brief Buffer for transmit message. */

	uint8_t max_size_buf_in;			/**	4. @brief Max lenght */

	uint8_t max_size_buf_out;			/**	5. @brief Max lenght */
	
	volatile uint8_t cur_size_buf_in;			/**	4. @brief lenght message in input buffer */

	volatile uint8_t cur_size_buf_out;			/**	5. @brief lenght message in output buffer */
	
	volatile uint8_t flag_tc_dma;		/**	6. @brief This flag shows transfer status message that put in @param LINE_PROTOCOL.buf_out.
											*	This flag can be set by @fn set_flag_tx and get from @fn get_flag_tx.
											*	Available values in @param LINE_FLAG_TX enum. */
}
data_protocol;

extern data_protocol data_resistance;

/* 6. */
uint8_t get_flag_tc_dma (data_protocol * _data_protocol);
void set_flag_tc_dma (data_protocol * _data_protocol, uint8_t flag_value);

/* 6. */
uint8_t get_cur_size_buf_in (data_protocol * _data_protocol);
void set_cur_size_buf_in (data_protocol * _data_protocol, uint8_t value);

/* 6. */
uint8_t get_flag_tc_dma (data_protocol * _data_protocol);
void set_flag_tc_dma (data_protocol * _data_protocol, uint8_t flag_value);

void send_message (data_protocol * _data_protocol);

extern inline void search_message (data_protocol * _data_protocol);
inline static void parse_message (data_protocol * _data_protocol);

static void prepare_data_to_transfer (void);

#endif
