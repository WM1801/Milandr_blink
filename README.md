Проект сделан на основе:
- https://www.youtube.com/watch?v=6IvoOdfX51c&list=PL9EFDdqdPIvzqJekgWcbWdKZSg2mB7H_A (https://dzen.ru/video/watch/624a003bb8d41826094fffde)
- https://habr.com/ru/articles/788776/
- https://support.milandr.ru/base/primenenie/programmirovanie-32-razryadnykh-mk/nachalo-raboty/50002/


Software pack для Keil MDK 5 + Standard Peripherals Library 1.2.0 от 02.02.2024 (https://support.milandr.ru/products/mikroskhemy_v_plastikovykh_korpusakh/k1986ve92qi)
Standard Peripherals Library 1.3.1 от 23.04.2024 (https://support.milandr.ru/products/mikroskhemy_v_plastikovykh_korpusakh/k1986ve92qi)


Файлы SPL/MDR32FxQI/MDR32FxQI_config.h добавлено: 
#if defined (__GNUC__) /* ARM GCC */
    #define IAR_SECTION(section)
    #define __RAMFUNC
#endif
Файл CMSIS/MDR32FxQI/DeviceSupport/MDR32F9Q2I/startup/system_MDR32F9Q2I.c добавлено:
#elif defined (__GNUC__) /* ARM GCC */
    extern uint32_t __Vectors;
    #define __VECTOR_TABLE_ADDRESS &__Vectors

Файл /SPL/MDR32FxQI/MDR32FxQI_config.h
- раскоментирован тип микроконтроллера  #define USE_MDR32F9Q2I

Файл /SPL/MDR32FxQI/src/IRQ_Handler_Template/MDR32F9Q2I_IT.c
- обработчики прерываний сделаны __attribute__((weak))

Файл /make_files/MDR32F9Qx.ld и /CMSIS/MDR32FxQI/DeviceSupport/MDR32F9Q2I/startup/arm/startup_MDR32F9Q2I.S
взяты из проекта https://www.youtube.com/watch?v=6IvoOdfX51c&list=PL9EFDdqdPIvzqJekgWcbWdKZSg2mB7H_A

Загрузка осуществлялась китайским Jlink V9
