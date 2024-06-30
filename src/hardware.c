#include "hardware.h"
#include "core_cm3.h"

#include "stdio.h"
#include <stdarg.h>
#include <string.h>


#define _PER_CLOCK_UART1_PORTA_EN	PER_CLOCK_PORTA_EN
#define _PER_CLOCK_UART2_PORTA_EN	PER_CLOCK_PORTF_EN
#define _UART1_GPIO_PORT			MDR_PORTA
#define _UART2_GPIO_PORT			MDR_PORTF
#define _UART1_RX_PIN_Pos			(6)
#define _UART1_TX_PIN_Pos			(7)
#define _UART2_RX_PIN_Pos			(0)
#define _UART2_TX_PIN_Pos			(1)


//------------------------------------------------- CLOCK -------------------------------------------------

static volatile uint32_t _tickCnt;

void HARDWARE_ClockInit(void)
{
	MDR_RST_CLK->HS_CONTROL |= RST_CLK_HS_CONTROL_HSE_ON;
	while(!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_HSE_RDY)) {}
	MDR_RST_CLK->CPU_CLOCK |= 0x2 << RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos;
	
	MDR_RST_CLK->PLL_CONTROL |= CLK_PLL_COEF << RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos;
	MDR_RST_CLK->PLL_CONTROL |= RST_CLK_PLL_CONTROL_PLL_CPU_ON;
	while(!(MDR_RST_CLK->CLOCK_STATUS & RST_CLK_CLOCK_STATUS_PLL_CPU_RDY)) {}
	
	//MDR_RST_CLK->CPU_CLOCK ^= RST_CLK_CPU_CLOCK_CPU_C2_SEL;
	MDR_RST_CLK->CPU_CLOCK |= 0x1 << RST_CLK_CPU_CLOCK_HCLK_SEL_Pos;
}

void HARDWARE_SysTickInit(uint32_t divider)
{
	SysTick->LOAD = (CLK_CPU_FREQ / divider) - 1;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	NVIC_EnableIRQ(SysTick_IRQn);
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

uint32_t HARDWARE_GetTick(void)
{
	return _tickCnt;
}

void HARDWARE_DelayMs(uint32_t wait)
{
	uint32_t tickStart = _tickCnt;
	while(_tickCnt < tickStart + wait);
}

void SysTick_Handler()
{
	_tickCnt++;
}

//------------------------------------------------- GPIO -------------------------------------------------

void GPIO_Init(void)
{
	MDR_RST_CLK->PER_CLOCK |= PER_CLOCK_PORTB_EN | PER_CLOCK_PORTC_EN | PER_CLOCK_PORTD_EN;

	BTN_PORT->ANALOG 	|= 1 << BTN_PIN_Pos;

	LED1_PORT->ANALOG 	|= 1 << LED1_PIN_Pos;
	LED2_PORT->ANALOG 	|= 1 << LED2_PIN_Pos;
	//LED3_PORT->ANALOG 	|= 1 << LED3_PIN_Pos;

	LED1_PORT->OE 		|= 1 << LED1_PIN_Pos;
	LED2_PORT->OE 		|= 1 << LED2_PIN_Pos;
	//LED3_PORT->OE 		|= 1 << LED3_PIN_Pos;

	LED1_PORT->PWR 		|= PORT_PWR_20NS << (LED1_PIN_Pos * 2);
	LED2_PORT->PWR 		|= PORT_PWR_20NS << (LED2_PIN_Pos * 2);
	//LED3_PORT->PWR 		|= PORT_PWR_20NS << (LED3_PIN_Pos * 2);
	
	//LED3_PORT->RXTX 	|= 1 << LED3_PIN_Pos;
}

void GPIO_WritePin(MDR_PORT_TypeDef * MDR_PORTx, uint32_t pinPos, bool state)
{
	if(state)
	{
		MDR_PORTx->RXTX |= 1 << pinPos; 
	}
	else
	{
		MDR_PORTx->RXTX &= ~(1 << pinPos); 
	}
}

void GPIO_TogglePin(MDR_PORT_TypeDef * MDR_PORTx, uint32_t pinPos)
{
		MDR_PORTx->RXTX ^= 1 << pinPos; 
}

bool GPIO_ReadPin(MDR_PORT_TypeDef * MDR_PORTx, uint32_t pinPos)
{
	uint32_t pinMask = (1 << pinPos);
	return (MDR_PORTx->RXTX & pinMask) == pinMask; 
}
	
//------------------------------------------------- UART -------------------------------------------------

bool myUART_Init(MDR_UART_TypeDef * MDR_UARTx, uint32_t baudRate)
{
	if(CLK_CPU_FREQ < (baudRate * 16)) 			{return false;}
	if(CLK_CPU_FREQ > (baudRate * 16 * 65535)) 	{return false;}
	
	uint32_t divider = CLK_CPU_FREQ / ((baudRate * 16) >> 6);
	uint32_t integerDivider    = divider >> 6;
	uint32_t fractionalDivider = divider & 0x3F;

	uint32_t realBaudRate  = (CLK_CPU_FREQ * ((1 << 6) / 16)) / ((integerDivider << 6) + fractionalDivider);
	uint32_t baudRateError = ((realBaudRate - baudRate) * 128) / baudRate;
	
	if (baudRateError > 2) {return false;}

	if(MDR_UARTx == MDR_UART1)
	{
		MDR_RST_CLK->PER_CLOCK	 	|= _PER_CLOCK_UART1_PORTA_EN;
		_UART1_GPIO_PORT->ANALOG	|= (1 << _UART1_RX_PIN_Pos) | (1 << _UART1_TX_PIN_Pos);
		_UART1_GPIO_PORT->FUNC		|= PORT_FUNC_REDEF << (_UART1_RX_PIN_Pos * 2);
		_UART1_GPIO_PORT->FUNC		|= PORT_FUNC_REDEF << (_UART1_TX_PIN_Pos * 2);
		_UART1_GPIO_PORT->OE		|= 1 << _UART1_TX_PIN_Pos;
		_UART1_GPIO_PORT->PWR		|= PORT_PWR_20NS << (_UART1_TX_PIN_Pos * 2);

		MDR_RST_CLK->PER_CLOCK 		|= PER_CLOCK_UART1_EN;
		MDR_RST_CLK->UART_CLOCK 	|= RST_CLK_UART_CLOCK_UART1_CLK_EN;
		NVIC_EnableIRQ(UART1_IRQn);
	}
	else if(MDR_UARTx == MDR_UART2)
	{
		MDR_RST_CLK->PER_CLOCK	 	|= _PER_CLOCK_UART2_PORTA_EN;
		_UART2_GPIO_PORT->ANALOG	|= (1 << _UART2_RX_PIN_Pos) | (1 << _UART2_TX_PIN_Pos);
		_UART2_GPIO_PORT->FUNC		|= PORT_FUNC_REDEF << (_UART2_RX_PIN_Pos * 2);
		_UART2_GPIO_PORT->FUNC		|= PORT_FUNC_REDEF << (_UART2_TX_PIN_Pos * 2);
		_UART2_GPIO_PORT->OE		|= 1 << _UART2_TX_PIN_Pos;
		_UART2_GPIO_PORT->PWR		|= PORT_PWR_20NS << (_UART2_TX_PIN_Pos * 2);

		MDR_RST_CLK->PER_CLOCK 		|= PER_CLOCK_UART2_EN;
		MDR_RST_CLK->UART_CLOCK 	|= RST_CLK_UART_CLOCK_UART2_CLK_EN;
		NVIC_EnableIRQ(UART2_IRQn);
	}
	
	MDR_UARTx->CR    &= ~UART_CR_UARTEN;
	MDR_UARTx->IBRD   = integerDivider;
	MDR_UARTx->FBRD   = fractionalDivider;
	MDR_UARTx->LCR_H |= UART_LCR_H_8BIT << UART_LCR_H_WLEN_Pos;
	//MDR_UARTx->LCR_H |= UART_LCR_H_PEN;
	//MDR_UARTx->LCR_H |= UART_LCR_H_EPS;
	MDR_UARTx->CR    |= UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
	MDR_UARTx->IMSC  |= UART_IMSC_RXIM;
  
	return 0;
}

void UART_Send(MDR_UART_TypeDef * MDR_UARTx, uint8_t * buf, uint16_t len)
{
	for(uint16_t i = 0; i < len; i++)
	{
		MDR_UARTx->DR = (buf[i] & (uint16_t)0x01FF);
		while(MDR_UARTx->FR & UART_FR_BUSY){}
	}
}

void UART_Printf(MDR_UART_TypeDef * MDR_UARTx, const char * str, ...)
{
	va_list args;
	va_start(args, str);
	
	// TODO: check buf size
	uint8_t sendBuf[200];
	vsprintf((char*)sendBuf, str, args);
	UART_Send(MDR_UARTx, sendBuf, strlen((char*)sendBuf));

	va_end(args);
}

void UART1_IRQHandler(void)
{
	if(MDR_UART1->RIS & UART_RIS_RXRIS)
	{
		MDR_UART1->DR;
	}
	MDR_UART1->ICR |= UART_ICR_RXIC;
}


void UART2_IRQHandler(void)
{
	if(MDR_UART2->RIS & UART_RIS_RXRIS)
	{
		MDR_UART2->DR;
	}
	MDR_UART2->ICR |= UART_ICR_RXIC;
}




