#ifndef __USART__H__
#define __USART__H__

/* Header khai báo các hàm cấu hình và gửi dữ liệu qua UART1 */

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void Usart_Int(uint32_t BaudRatePrescaler);
void USART1_send_byte(uint8_t byte);
void USART_Send_bytes(const char *Buffer, uint16_t Length);
#endif //__USART__H__
