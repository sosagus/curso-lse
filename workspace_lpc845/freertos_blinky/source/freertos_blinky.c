#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"

void task_hello (void *params) {
	while(1){
		printf("Skibidi toilet");
		vTaskDelay(500);
	}
}

int main(void) {
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    xTaskCreate{
    	task_hello;
		"skibidi";
		configminimal_stack_size;
		NULL;
		1;
		NULL;
    }

    vTaskStartScheduler();

    while(){
    }
    return 0 ;
}
