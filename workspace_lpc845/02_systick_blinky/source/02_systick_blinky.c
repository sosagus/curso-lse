#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"

#define LED_BLUE	1
#define D1	0

int main(void) {
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    BOARD_BootClockFRO24M();

    gpio_pin_config_t out_config = { kGPIO_DigitalOutput, 1 };

    GPIO_PortInit(GPIO, 1);
    GPIO_PinInit(GPIO, 1, 1, &out_config);
    GPIO_PortInit(GPIO, 0);
    GPIO_PinInit(GPIO, 0, 29, &out_config);

    SysTick_Config(SystemCoreClock / 1000);

    return 0;
}

void SysTick_Handler(void) {
    static uint16_t i = 0;
    i++;
    if(i == 1500) {
    	i = 0;
    	GPIO_PinWrite(GPIO, 0, 29, !GPIO_PinRead(GPIO, 0, 29));
    }
    if (i % 500 == 0) {
    	GPIO_PinWrite(GPIO, 1, 1, !GPIO_PinRead(GPIO, 1, 1));
    }
}
