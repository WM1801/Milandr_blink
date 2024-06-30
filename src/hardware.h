#include "MDR32F9Q2I.h"
#include "hw_defs.h"


#define CLK_QRZ_FREQ 	(8000000)
#define CLK_CPU_FREQ 	(32000000)
#define CLK_PLL_COEF	((CLK_CPU_FREQ / CLK_QRZ_FREQ) - 1)

#define BTN_PORT	    (MDR_PORTD)
#define BTN_PIN_Pos	    (5)
#define LED1_PORT	    (MDR_PORTB)
#define LED1_PIN_Pos	(1)
#define LED2_PORT	    (MDR_PORTC)
#define LED2_PIN_Pos	(0)
//#define LED3_PORT	    (MDR_PORTC)
//#define LED3_PIN_Pos	(2)


void		HARDWARE_ClockInit(void);
void 		HARDWARE_SysTickInit(uint32_t divider);
uint32_t	HARDWARE_GetTick(void);
void		HARDWARE_DelayMs(uint32_t wait);

void 		GPIO_Init(void);
void        GPIO_WritePin(MDR_PORT_TypeDef * MDR_PORTx, uint32_t pinPos, bool state);
void        GPIO_TogglePin(MDR_PORT_TypeDef * MDR_PORTx, uint32_t pinPos);
bool        GPIO_ReadPin(MDR_PORT_TypeDef * MDR_PORTx, uint32_t pinPos);

uint8_t		UART_Init(MDR_UART_TypeDef * MDR_UARTx, uint32_t baudRate);
void 		UART_Send(MDR_UART_TypeDef * MDR_UARTx, uint8_t * buf, uint16_t len);
void        UART_Printf(MDR_UART_TypeDef * MDR_UARTx, const char * str, ...);
