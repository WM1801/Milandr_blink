#ifndef _HW_DEFS_H_
#define _HW_DEFS_H_

#define bool	uint8_t
#define false 	(0)
#define true	(1)

//CLK
#define PER_CLOCK_UART1_EN_Pos		(6)
#define PER_CLOCK_UART1_EN			((uint32_t)(1 << PER_CLOCK_UART1_EN_Pos))
#define PER_CLOCK_UART2_EN_Pos		(7)
#define PER_CLOCK_UART2_EN			((uint32_t)(1 << PER_CLOCK_UART2_EN_Pos))
#define PER_CLOCK_PORTA_EN_Pos		(21)
#define PER_CLOCK_PORTA_EN			((uint32_t)(1 << PER_CLOCK_PORTA_EN_Pos))
#define PER_CLOCK_PORTB_EN_Pos		(22)
#define PER_CLOCK_PORTB_EN			((uint32_t)(1 << PER_CLOCK_PORTB_EN_Pos))
#define PER_CLOCK_PORTC_EN_Pos		(23)
#define PER_CLOCK_PORTC_EN			((uint32_t)(1 << PER_CLOCK_PORTC_EN_Pos))
#define PER_CLOCK_PORTD_EN_Pos		(24)
#define PER_CLOCK_PORTD_EN			((uint32_t)(1 << PER_CLOCK_PORTD_EN_Pos))
#define PER_CLOCK_PORTE_EN_Pos		(25)
#define PER_CLOCK_PORTE_EN			((uint32_t)(1 << PER_CLOCK_PORTE_EN_Pos))
#define PER_CLOCK_PORTF_EN_Pos		(29)
#define PER_CLOCK_PORTF_EN			((uint32_t)(1 << PER_CLOCK_PORTF_EN_Pos))


//GPIO
#define PORT_PWR_OFF		(0x0)
#define PORT_PWR_100NS		(0x1)
#define PORT_PWR_20NS		(0x2)
#define PORT_PWR_10NS		(0x3)

#define PORT_FUNC_PRT		(0x0)
#define PORT_FUNC_MAIN		(0x1)
#define PORT_FUNC_ALTER		(0x2)
#define PORT_FUNC_REDEF		(0x3)

//UART
#define UART_LCR_H_5BIT		(0x0)
#define UART_LCR_H_6BIT		(0x1)
#define UART_LCR_H_7BIT		(0x2)
#define UART_LCR_H_8BIT		(0x3)

#endif
