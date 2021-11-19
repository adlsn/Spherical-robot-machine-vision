#ifndef _UART1_H_
#define _UART1_H_
#include "stm32f4xx.h"
#include "sys.h"

void USART1_IRQ_Isr(void);
void USART1_Init(void);
u8 USART1_RecBuffer(u8 *buffer);
u8 USART1_SendBuffer(u8 *buffer, u16 n);


#endif