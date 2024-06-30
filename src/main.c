#define SPL

#ifdef SPL
#include <MDR32FxQI_config.h>
#include <MDR32F9Q2I.h>
#include <MDR32FxQI_rst_clk.h>
#include <MDR32FxQI_port.h>
#include <MDR32FxQI_rst_clk.h>

void Delay(int waitTicks);

int main() {
  // Заводим структуру конфигурации вывода(-ов) порта GPIO
  PORT_InitTypeDef GPIOInitStruct;
  // Включаем тактирование порта C
  RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB, ENABLE);
  // Инициализируем структуру конфигурации вывода(-ов) порта значениями по
  // умолчанию
  PORT_StructInit(&GPIOInitStruct);
  // Изменяем значения по умолчанию на необходимые нам настройки
  GPIOInitStruct.PORT_Pin = PORT_Pin_1;
  GPIOInitStruct.PORT_OE = PORT_OE_OUT;
  GPIOInitStruct.PORT_SPEED = PORT_SPEED_SLOW;
  GPIOInitStruct.PORT_MODE = PORT_MODE_DIGITAL;
  // Применяем заполненную нами структуру для PORTB.
  PORT_Init(MDR_PORTB, &GPIOInitStruct);
  //
  while (1) {
    /*if (PORT_ReadInputDataBit (MDR_PORTB, PORT_Pin_1) == 0){
      PORT_SetBits(MDR_PORTC, PORT_Pin_2); // светоидиот на нашей плате
    }else{
      PORT_ResetBits(MDR_PORTC, PORT_Pin_2);
    }*/
    // Задержка
	PORT_ResetBits(MDR_PORTB, PORT_Pin_1);
    Delay(1000000);
	PORT_SetBits(MDR_PORTB, PORT_Pin_1);
	Delay(1000000);
  }
}

// Простейшая функция задержки, позднее мы заменим ее на реализацию через таймер
void Delay(int waitTicks) {
  int i;
  for (i = 0; i < waitTicks; i++) {
    __NOP();
  }
}


#else

#include "hardware.h"


//static bool 		_startFlag = false;
//static uint32_t 	_itrCnt = 0;

void Delay(int waitTicks) {
  int i;
  for (i = 0; i < waitTicks; i++) {
    __NOP();
  }
}


int main(void)
{
	GPIO_Init();
	HARDWARE_SysTickInit(10);
	//UART_Init(MDR_UART2, 115200 * 2);
	
	while(1)
	{
		//GPIO_TogglePin(LED1_PORT, LED1_PIN_Pos);
		GPIO_TogglePin(LED2_PORT, LED2_PIN_Pos);
		Delay(500000) ; 
		/*if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
		{
			GPIO_TogglePin(LED3_PORT, LED3_PIN_Pos);
			UART_Printf(MDR_UART2, "\r\nTest iteration: %u", _itrCnt++);
		}

		static bool _btnFlag = false;

		if(!GPIO_ReadPin(BTN_PORT, BTN_PIN_Pos) && !_btnFlag)
		{
			if(!_startFlag)
			{
				_startFlag = true;
				GPIO_WritePin(LED2_PORT, LED2_PIN_Pos, true);
				HARDWARE_ClockInit();
			}
			else
			{	
				GPIO_TogglePin(LED1_PORT, LED1_PIN_Pos);
				GPIO_TogglePin(LED2_PORT, LED2_PIN_Pos);
				MDR_RST_CLK->CPU_CLOCK ^= RST_CLK_CPU_CLOCK_CPU_C2_SEL;
			}
			_btnFlag = true;
		}
		
		if(GPIO_ReadPin(BTN_PORT, BTN_PIN_Pos) && _btnFlag)
		{
			_btnFlag = false;
		}*/
	}
}

#endif