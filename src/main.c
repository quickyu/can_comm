#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <RTL.h>   

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_can.h"
#include "misc.h"

   
#define	SERIAL_BUF_LEN		100  

os_mbx_declare (serial_send_buf, SERIAL_BUF_LEN);
os_mbx_declare (serial_recv_buf, SERIAL_BUF_LEN);
static int serial_send_tag = 0;


#define CAN_MSG_CNT			32

_declare_box(mem_pool, sizeof(CanRxMsg), CAN_MSG_CNT);
os_mbx_declare(can_msg_queue, CAN_MSG_CNT);


OS_MUT serial_send_mutex;

static int comm_led = 0;



void NVIC_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the CAN Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void init_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOD Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIOA->BSRR = (1<<7)|(1<<6);
}

void init_serial(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	os_mbx_init(serial_send_buf, sizeof(serial_send_buf));
	os_mbx_init(serial_recv_buf, sizeof(serial_recv_buf));
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
							RCC_APB2Periph_AFIO | 
							RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART1 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	/* Configure USART1 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART1 configuration ------------------------------------------------------*/
	/* USART1 configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure USART1 */
	USART_Init(USART1, &USART_InitStructure);
	
  	/* Enable USART1 Receive and Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);
}

int write_serial(uint8_t *buf, int len, int timeout)  
{     
	int i;
	    
	for (i = 0; i < len; i++) {
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		
		if (!serial_send_tag) {
			serial_send_tag = 1;
			USART_SendData(USART1, buf[i]);
		} else {
			if (os_mbx_send(serial_send_buf, (void *)buf[i], 0) != OS_R_OK) { 
				USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
				break;
			}	
		}	

		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	}

	return i;
}

int read_serial(uint8_t *buf, int len, int timeout)  
{                 
    void *data;
	int i;

	for (i = 0; i < len; i++) {
		if (os_mbx_wait(serial_recv_buf, &data, timeout) == OS_R_TMO)
			return i;

 		buf[i] = (uint8_t)data;
	}

	return i;
} 

__irq void USART1_IRQHandler(void)
{
	static uint8_t recv_data;
	static void *send_data;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		recv_data = USART_ReceiveData(USART1);
		if (isr_mbx_check(serial_recv_buf) != 0)
			isr_mbx_send(serial_recv_buf, (void *)recv_data);
	}

	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) { 
		if (isr_mbx_receive(serial_send_buf, &send_data) == OS_R_MBX)
			USART_SendData(USART1, (uint8_t)send_data);
  		else {
		    serial_send_tag = 0;
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  		}	
	}	
}

void init_can(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

  	os_mbx_init(can_msg_queue, sizeof(can_msg_queue));
	
	/* GPIO clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure CAN pin: RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	/* Configure CAN pin: TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	/* CANx Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

 	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	/* CAN Baudrate = 250kbps*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
	CAN_InitStructure.CAN_Prescaler = 24;
	CAN_Init(CAN1, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

__irq void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg *rx_msg;

	rx_msg = _alloc_box(mem_pool);
	if (rx_msg == NULL)
		return;

	comm_led = 1;
	CAN_Receive(CAN1, CAN_FIFO0, rx_msg);
	isr_mbx_send(can_msg_queue, rx_msg);
}

uint8_t cal_check(uint8_t *frame, int len)
{
	uint8_t check = 0;

    while (len--) {
		check ^= *frame++;
	}
	
    return check;
}

#define FRAME_BUF_LEN		20

__task void receive_can_data(void)
{
	CanRxMsg *rx_msg;
	uint8_t buff[FRAME_BUF_LEN];
	int frame_len;
	
	buff[0] = 0x7e;
	buff[2] = 0x01;
			
	while(1) {
		os_mbx_wait(can_msg_queue, (void **)&rx_msg, 0xffff);

		if (rx_msg->IDE == CAN_ID_EXT) {
			if (rx_msg->RTR == CAN_RTR_DATA) {
				frame_len = rx_msg->DLC+4+4+1;
				buff[1] = frame_len;
				buff[3] = 0;
				buff[4] = rx_msg->ExtId&0xff;
				buff[5] = (rx_msg->ExtId>>8)&0xff;
				buff[6] = (rx_msg->ExtId>>16)&0xff;
				buff[7] = (rx_msg->ExtId>>24)&0xff;
				
				if (rx_msg->DLC > 0)
					memcpy(&buff[8], rx_msg->Data, rx_msg->DLC);
			} else {
				frame_len = 4+4+1;
				buff[1] = frame_len;
				buff[3] = 1;
				buff[4] = rx_msg->ExtId&0xff;
				buff[5] = (rx_msg->ExtId>>8)&0xff;
				buff[6] = (rx_msg->ExtId>>16)&0xff;
				buff[7] = (rx_msg->ExtId>>24)&0xff;
			}

			buff[frame_len-1] = cal_check(buff, frame_len-1);

			os_mut_wait(serial_send_mutex, 0xffff);
			write_serial(buff, frame_len, 5);
			os_mut_release(serial_send_mutex);
		}	

		_free_box(mem_pool, rx_msg);
	}	
}

__task void serial_handler(void)
{
	uint8_t buff[FRAME_BUF_LEN];
	const uint8_t resp[5] = {0x7E, 0x05, 0x04, 0x01, 0x7E}; 
	int len;
	CanTxMsg tx_msg;
	uint32_t filter_id, filter_mask;
	CAN_FilterInitTypeDef can_filter;
	
	while (1) {
_restart:		
		if (read_serial(buff, 1, 5) == 0)
			goto _restart;
		if (buff[0] != 0x7e)
			goto _restart;

		if (read_serial(buff+1, 1, 5) == 0)
			goto _restart;
		if (buff[1] > FRAME_BUF_LEN)
			goto _restart;

		len = buff[1];
		
		if (read_serial(buff+2, len-2, 5) != len-2)
			goto _restart;
		
		switch (buff[2]) {
		case 2:
			if (buff[3] == 0) {
				tx_msg.IDE = CAN_ID_EXT;
				tx_msg.RTR = CAN_RTR_DATA;
				tx_msg.ExtId = ((uint32_t)buff[7]<<24) + ((uint32_t)buff[6]<<16)
								+ ((uint32_t)buff[5]<<8) + (uint32_t)buff[4];
				tx_msg.DLC = len-4-5;
				memcpy(tx_msg.Data, &buff[8], len-4-5);
			} else {
				tx_msg.IDE = CAN_ID_EXT;
				tx_msg.RTR = CAN_RTR_REMOTE;
				tx_msg.ExtId = ((uint32_t)buff[7]<<24) + ((uint32_t)buff[6]<<16)
								+ ((uint32_t)buff[5]<<8) + (uint32_t)buff[4];
				tx_msg.DLC = 0;
			}

			CAN_Transmit(CAN1, &tx_msg);

			os_mut_wait(serial_send_mutex, 0xffff);
			write_serial((uint8_t *)resp, 5, 5);
			os_mut_release(serial_send_mutex);
			
			break;
			
		case 3:
			filter_mask = ((uint32_t)buff[6]<<24) + ((uint32_t)buff[5]<<16)
								+ ((uint32_t)buff[4]<<8) + (uint32_t)buff[3];
			filter_mask <<= 3;

			filter_id = ((uint32_t)buff[10]<<24) + ((uint32_t)buff[9]<<16)
								+ ((uint32_t)buff[8]<<8) + (uint32_t)buff[7];
			filter_id <<= 3;

			/* CAN filter init */
			can_filter.CAN_FilterNumber = 0;
			can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
			can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
			can_filter.CAN_FilterIdHigh = filter_id/65536;
			can_filter.CAN_FilterIdLow = filter_id%65536;
			can_filter.CAN_FilterMaskIdHigh = filter_mask/65536;
			can_filter.CAN_FilterMaskIdLow = filter_mask%65536;
			can_filter.CAN_FilterFIFOAssignment = 0;
			can_filter.CAN_FilterActivation = ENABLE;
			CAN_FilterInit(&can_filter);

			os_mut_wait(serial_send_mutex, 0xffff);
			write_serial((uint8_t *)resp, 5, 5);
			os_mut_release(serial_send_mutex);
			
			break;
			
		default: goto _restart;	
		}	
	}	
}

__task void blink(void)
{
	unsigned int counter = 0;

	while(1) {
		if (++counter == 50) {
			counter = 0;

			GPIOA->ODR ^= 1<<7;
		}	

		if (comm_led) {
			GPIOA->BRR = 1<<6;
			comm_led = 0;
		} else
			GPIOA->BSRR = 1<<6;
		
 		os_dly_wait(1);
	}
}

__task void initialize(void) 
{
	if (_init_box(mem_pool, sizeof(mem_pool), sizeof(CanRxMsg)) != 0) {
		for (;;);
	}

	os_mut_init(serial_send_mutex);

	init_gpio();
	init_serial(); 
	init_can();
	
	os_tsk_create(blink, 1);
	os_tsk_create(receive_can_data, 11);
	os_tsk_create(serial_handler, 10);
	
	os_tsk_delete_self ();     
}

int main (void) 
{   
	NVIC_config();
		
    os_sys_init_prio(initialize, 50);           

	for (;;);

	return 0;
}


