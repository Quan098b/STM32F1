#include "usart.h"

/*
 * File: usart.c
 * Chức năng:
 * - Cấu hình UART1 để gửi log/debug từ STM32F103 ra máy tính.
 * - TX dùng chân PA9, RX dùng chân PA10.
 * - Cung cấp hàm gửi 1 byte và gửi cả chuỗi byte.
 */

void Usart_Int(uint32_t BaudRatePrescaler)
{
	GPIO_InitTypeDef GPIO_usartx;
	USART_InitTypeDef Usart_X;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	  //USART1_TX   PA.9
	GPIO_usartx.GPIO_Pin = GPIO_Pin_9;
	GPIO_usartx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_usartx.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_usartx);
	//USART1_RX	  PA.10
	GPIO_usartx.GPIO_Pin = GPIO_Pin_10;
	GPIO_usartx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_usartx);

	Usart_X.USART_BaudRate=BaudRatePrescaler;
	Usart_X.USART_WordLength=USART_WordLength_8b;
	Usart_X.USART_StopBits=USART_StopBits_1;
	Usart_X.USART_Parity=USART_Parity_No;
	Usart_X.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	Usart_X.USART_Mode= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &Usart_X);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);
}

void USART1_send_byte(uint8_t byte)
{
	while(USART_GetFlagStatus(USART1,USART_SR_TXE)==RESET);
	USART1->DR=byte;
}

void USART_Send_bytes(const char *Buffer, uint16_t Length)
{
	uint16_t i=0;
	while(i<Length)
	{
		USART1_send_byte((uint8_t)Buffer[i++]);
	}
}
