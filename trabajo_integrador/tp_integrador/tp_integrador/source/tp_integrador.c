#include <stdio.h>
#include "board.h"
#include "Wrappers.h"
#include "FreeRTOS.h"
#include "app_tasks.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
#include "task.h"
#include "pin_mux.h"
#include "clock_config.h"

#define BH1750_ADDR	0x5c
#define S1 16
#define S2 25
#define USER_BTN 4
#define LED_B 29
#define ADC_POT_CH 0

int main(void) {

	BOARD_BootClockFRO30M();
	BOARD_InitDebugConsole();
	    gpio_pin_config_t config = { kGPIO_DigitalOutput, 0 };
	    GPIO_PortInit(GPIO, 0);
	    GPIO_PinInit(GPIO, 0, 28, &config);
	    GPIO_PinWrite(GPIO, 0, 28, 0);


	xTaskCreate(task_Inicio, "Inicio", tskInicio_STACK, NULL, tskInicio_PRIORITY, NULL);

	xTaskCreate(task_Tiempo, "Tiempo", tskTiempo_STACK, NULL, tskTiempo_PRIORITY, NULL);

	xTaskCreate(task_Consola, "Consola", tskConsola_STACK, NULL, tskConsola_PRIORITY, NULL);

	xTaskCreate(task_bh1750, "BH1750", tskBH1750_STACK, NULL, tskBH1750_PRIORITY, NULL);

	xTaskCreate(task_Btn, "Boton", tskBtn_STACK, NULL, tskBtn_PRIORITY, NULL);

	xTaskCreate(task_Displaywrite, "Write", tskDisplaywrite_STACK, NULL, tskDisplaywrite_PRIORITY, &handle_display);

	xTaskCreate(task_ADCread, "ADC", tskADCread_STACK, NULL, tskADCread_PRIORITY, NULL);

	xTaskCreate(task_PWM, "PWM", tskPWM_STACK, NULL, tskPWM_PRIORITY, NULL);

	vTaskStartScheduler();

    while(1);
    return 0;
}