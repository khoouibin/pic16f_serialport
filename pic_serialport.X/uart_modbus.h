// ---------------------------------------------------------------------------
//  Filename: UART_modbus.h
//  Created by: Khoo UiBin
//  Date:  June 09, 2021.
//  Orisol (c)
// ---------------------------------------------------------------------------

#ifndef _UART_MODBUS_H_
#define _UART_MODBUS_H_
#include <stdbool.h>
#define Test_1 LATAbits.LATA0
#define Test_2 LATAbits.LATA1
#define uart_rx_size 160
#define uart_tx_size 160
// #define modbus_SilentInt      175   // Silent Interval: 3/4 ms. 1cnt = 4.26usec.
// #define modbus_SilentInt      93   // Silent Interval: 3/4 ms. 1cnt = 8usec.
#define modbus_SilentH 0xFD // Silent Interval: 3/4 ms. 1cnt = 1usec.
#define modbus_SilentL 0x11 // 0.75ms = 750 usec -> 65535 - 750 = 64785d = FD11

#define modbus_resp_0ms 50	  // 42.6usec.
#define modbus_resp_1ms 1000  // 1ms
#define modbus_resp_10ms 2340 // 10ms
#define REPLY_PERIOD_MIN 10	  // 10ms

#define SLAVE_ADDR_DEF 10

#define POS_RESPONSE 0x00
#define NEG_RESPONSE 0x7F

#define MIN_VALID_SIZE 6
#define MAX_STITAB_SIZE 122 //(61)

typedef enum
{
	RTC_RESET = 0x11,
	READ_PARAMETER = 0x22,
	WRITE_PARAMETER = 0x2E,
	TEST_PRESENT = 0x3E,
} SID_t;

typedef enum
{
	RESET_RTC = 0x01,
	RESET_USB = 0x02,
} SUBFUNC_RESET_t;

typedef enum
{
	READ_STIMOV_TYP = 0x01,
	READ_STIMOV_TABSIZE = 0x02,
	READ_STIMOV_TABLE = 0x03,
	READ_STIMOV_MINI = 0x04,
} SUBFUNC_READ_PARM_t;

typedef enum
{
	WRITE_STIMOV_TYP = 0x01,
	WRITE_STIMOV_TABSIZE = 0x02,
	WRITE_STIMOV_TABLE = 0x03,
	WRITE_STIMOV_MINI = 0x04,
} SUBFUNC_WRITE_PARM_t;

typedef enum
{
	GeneralReject = 0x10,
	SIDnotSupport = 0x11,
	SubFuncNotSupport = 0x12,
	IncorrectLengFormat = 0x13,
	IncorrectCRC = 0x14,
	BusyRepeatRequest = 0x21,
	ConditionsNotCorrect = 0x22,
	OutOfRange = 0x31,
	SecurityAccessDenied = 0x33,
	InvalidKey = 0x35,
	ExceedNumOfAttempts = 0x36,
	RequireTimeDelayNotExpired = 0x37,
} NRC_t;

typedef enum
{
	state_idle = 0,
	state_receiving = 1,
} modbus_idle_t;

typedef enum
{
	listen_mode = 0,
	echo_mode,
	normal_mode,
	periodic_mode,
	maxnum_mode
} modbus_mode_t;

typedef enum
{
	no_filter = 0,
	slave_addr_filter,
	maxnum_filter
} modbus_loopback_t; // loopback reply select.

typedef enum
{
	status_rx = 0,
	status_wait = 1,
	status_tx = 2,
} modbus_status_t;

typedef struct
{
	unsigned char rxBuf[uart_rx_size];
	unsigned char *ptr_rxBuf;
	unsigned char rxSize;
} modbus_rxbuf_t;

typedef struct
{
	unsigned char txBuf[uart_tx_size];
	unsigned char *ptr_txBuf;
	unsigned char txSize;
	unsigned char send_num;
} modbus_txbuf_t;

typedef struct
{
	unsigned char tempBuf[uart_tx_size];
	unsigned char *ptr_tempBuf;
	unsigned char tempBufSize;
} modbus_tempbuf_t;

typedef enum
{
	silent_wait = 0,
	response_delay = 1,
} waiting_typ_t;

typedef struct
{
	waiting_typ_t wait_type;
	char delay_ms; // max: 127(ms) -> 127 * 234 = 29718, 29718*4.26usec =126.6ms.
} modbus_wait_t;

typedef struct
{
	char slave_addr;
	modbus_mode_t mode;
	modbus_loopback_t loopback_filter;
	modbus_status_t status;
	modbus_wait_t wait_ctrl;
	modbus_rxbuf_t rx_ctrl;
	modbus_txbuf_t tx_ctrl;
} uart_modbus_status_t;

typedef union
{
	unsigned short word;
	unsigned char u8[2];
	struct
	{
		unsigned LB : 8;
		unsigned HB : 8;
	} bytes;
} u16_union_t;

/*
typedef struct
{
	unsigned char *array;
	size_t used;
	size_t size;
}
Array_t;*/

void UART1_modbus_init(void);
void UART1_module_init(void);
void Tmr1_module_init(void);

char set_modbus_mode(modbus_mode_t mode);
modbus_mode_t get_modbus_mode(void);
char set_modbus_slave_addr(char addr);
char get_modbus_slave_addr(void);
char get_modbus_resp_delay_ms(void);
void set_modbus_resp_delay_ms(char delay_ms);
char set_modbus_loopback_filter(modbus_loopback_t filter_typ);
modbus_loopback_t get_modbus_loopback_filter(void);

// void tx_test(void);
waiting_typ_t get_Tmr1_WaitType(void);
void Tmr1_TypeSetting(waiting_typ_t type);
void delay_ms_to_Tmr1Count(char delay_ms, unsigned char *tmr1H, unsigned char *tmr1L);
void Tmr1_Off(void);

void reset_modbus_status_ptr(uart_modbus_status_t *status);

void txBuf_load(unsigned char *txbuf, unsigned char leng);
char txBuf_send(void);

char rxBuf_LoopbackMode_check(unsigned char *data_buf);
char rxBuf_check_SlaveAddr(unsigned char *data_buf);

char rxBuf_NormalMode_check(unsigned char *data_buf, unsigned char leng, NRC_t *NRC_fbk);
char rxBuf_check_checksum(unsigned char *data_buf, unsigned char leng);

char rxBuf_check_SID(unsigned char *data_buf);
char rxBuf_check_SubFunc(unsigned char *data_buf);
char rxBuf_check_DataSize(unsigned char *data_buf, unsigned char leng);

unsigned short Cal_CRC16(unsigned char *str_in, unsigned char input_len);

char execute_modbus_cmd(unsigned char *data_buf, unsigned char leng, modbus_tempbuf_t *res_fbk);
void pushback_modbus_tempbuf(modbus_tempbuf_t *buf, unsigned char data);

void tx_test(void);
modbus_idle_t isReceiveIdle(void);

void UART_Rx(unsigned char uart_byte);
void UART_Tx(unsigned char uart_byte);
void Tmr1_Process(void);
#endif