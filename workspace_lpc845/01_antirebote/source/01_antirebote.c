#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

#define ledr	2
#define user	04

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    gpio_pin_config_t out_config = { kGPIO_DigitalOutput, 1 };
    gpio_pin_config_t in_config = {kGPIO_DigitalInput};
    GPIO_PortInit(GPIO, 0);
    GPIO_PinInit(GPIO, 0, user, &in_config);
    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, ledr, &out_config);

    while(1){
    	if (!GPIO_PinRead(GPIO, 0, user)) {
    		GPIO_PinWrite(GPIO, 1, ledr, 0);
    		for (uint32_t i = 0; i < 200000; i++);
    	} else {
    		GPIO_PinWrite(GPIO, 1, ledr, 1);
    	}
    }
    return 0 ;
}
