#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

#define luz 11

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    gpio_pin_config_t out_config = { kGPIO_DigitalOutput, 0 };
    GPIO_PortInit(GPIO, 0);
    GPIO_PinInit(GPIO, 11, luz, &out_config);

    GPIO_PinWrite(GPIO, 0, luz, 1);
}
