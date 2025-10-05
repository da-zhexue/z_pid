#ifndef __UART_H__
#define __UART_H__

#include "main.h"
#include "pid.h"
#include <stdio.h>
#include <stdarg.h>

#define REC_MESSAGE_LEN 55
#define SEND_MESSAGE_LEN 32

typedef struct {
    uint8_t type_id; 
    union {
        int16_t  i16_val;
        uint16_t u16_val;
        int32_t  i32_val;
        uint32_t u32_val;
        float    f_val;
        uint8_t  u8_val;
    } value;
} custom_field_t;

extern float PID_V[3], IOUT_MAX_V, OUT_MAX_V;
extern float PID_P[3], IOUT_MAX_P, OUT_MAX_P;
extern uint8_t uart3_rx_buffer[REC_MESSAGE_LEN];
extern uint8_t uart3_tx_buffer[SEND_MESSAGE_LEN];

extern int running;

void Send_code(uint8_t *buffer);
void uart_init(void);
int myprintf(const char *format, ...);

#endif
