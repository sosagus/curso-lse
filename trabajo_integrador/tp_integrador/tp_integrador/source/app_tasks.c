#include "app_tasks.h"
#include "fsl_debug_console.h"

xQueueHandle queue_display1;
xQueueHandle queue_display2;
xQueueHandle queue_adc;
xQueueHandle queue_lux;
xQueueHandle queue_pwm;
xQueueHandle queue_tiempo;

TaskHandle_t handle_display;

void task_Inicio(void *params) {
	wrapper_gpio_init(0); // Inicio GPIO
	wrapper_adc_init(); // Configuracion ADC
	wrapper_display_init(); // Configuracion display
	wrapper_btn_init(); // Configuracion botones
	wrapper_pwm_init(); // Inicio PWM
	wrapper_i2c_init(); // Inicio I2C
	wrapper_bh1750_init(); // Inicio BH1750

	queue_adc = xQueueCreate(1, sizeof(adc_data_t));
	queue_lux = xQueueCreate(1, sizeof(uint8_t));
	queue_pwm = xQueueCreate(1, sizeof(uint8_t));
	queue_tiempo = xQueueCreate(1, sizeof(uint32_t));
	queue_display1 = xQueueCreate(1, sizeof(display_t));
	queue_display2 = xQueueCreate(1, sizeof(display_t));
	vTaskDelete(NULL);
}

void task_ADCread(void *params) {
	while(1) {
		ADC_DoSoftwareTriggerConvSeqA(ADC0); // Inicio de conversion
		vTaskDelay(250); // Bloqueo la tarea por 250 ms
	}
}

void task_Tiempo(void *params){
	uint32_t time = 0;
	while(1){
		vTaskDelay(1);
		time++;
		xQueueOverwrite(queue_tiempo, &time);
	}
}

void task_bh1750(void *params) {
		CLOCK_Select(kI2C1_Clk_From_MainClk); // Inicializo clock de I2C1
		CLOCK_EnableClock(kCLOCK_Swm);
	    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SDA, kSWM_PortPin_P0_27); // Asigno funciones de I2C1 al pin 27
	    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SCL, kSWM_PortPin_P0_26); // Asigno funciones de I2C1 al pin 26
	    CLOCK_DisableClock(kCLOCK_Swm);

	    i2c_master_config_t config; // Configuracion de master para el I2C con 400 KHz de clock
	    I2C_MasterGetDefaultConfig(&config);
	    config.baudRate_Bps = 400000;
	    I2C_MasterInit(I2C1, &config, SystemCoreClock); // Usa el clock del sistema de base para generar el de la comunicacion

		if(I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write) == kStatus_Success) {
			uint8_t cmd = 0x01; // Comando de power on
			I2C_MasterWriteBlocking(I2C1, &cmd, 1, kI2C_TransferDefaultFlag);
			I2C_MasterStop(I2C1);
		}
		if(I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write) == kStatus_Success) {
			uint8_t cmd = 0x10; // Comando de medicion continua a 1 lux de resolucion
			I2C_MasterWriteBlocking(I2C1, &cmd, 1, kI2C_TransferDefaultFlag);
			I2C_MasterStop(I2C1);
		}

		while(1) {
			if(I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Read) == kStatus_Success) { // Lectura del sensor
				uint8_t res[2] = {0};
				I2C_MasterReadBlocking(I2C1, res, 2, kI2C_TransferDefaultFlag);
				I2C_MasterStop(I2C1);
				float lux = ((res[0] << 8) + res[1]) / 1.2;
				uint8_t percentLux = (lux * 100.00) / 30000.00;
				xQueueOverwrite(queue_lux, &percentLux);
			}
	    }
}

void task_Consola(void *params){
	uint32_t time = 0;
	uint8_t lux = 0;
	display_t val = 50;
	uint8_t pwm = 0;
	PRINTF("Tiempo transcurrido | Luz | SP | PWM\n");
	while(1) {
		xQueueReceive(queue_tiempo, &time, portMAX_DELAY);
		xQueueReceive(queue_lux, &lux, portMAX_DELAY);
		xQueuePeek(queue_display2, &val, portMAX_DELAY);
		xQueuePeek(queue_pwm, &pwm, portMAX_DELAY);

		PRINTF("  %d    %d    %d    %d\n",
				time,
				lux,
				val,
				pwm
				);
		vTaskDelay(1000);

	}
}

void task_Btn(void *params) {
	display_t val = 50;
	display_t change = 0;
	uint8_t lux = 0;

	while(1) {
		xQueueReceive(queue_lux, &lux, portMAX_DELAY);
		if(!wrapper_btn_get(USR_BTN)) { // Verifico que boton se toco
			vTaskDelay(20); // Antirebote
			if(change == 1){
				change = 0;
			}
			else{
				change = 1;
			}
		}
		if(!wrapper_btn_get(S1_BTN)) {
			vTaskDelay(20); // Antirebote
			if(!wrapper_btn_get(S1_BTN) && val < 75) {
				val++;
			}
		}
		if(!wrapper_btn_get(S2_BTN)) {
			vTaskDelay(20); // Antirebote
			if(!wrapper_btn_get(S2_BTN) && val > 25) {
				val--;
			}
		}
		xQueueOverwrite(queue_display1, &change);
		xQueueOverwrite(queue_display2, &val);
		vTaskDelay(200);
	}
}

void task_Displaywrite(void *params) {
	display_t val = 50;
	display_t change = 0;
	uint8_t lux = 0;
	uint8_t show = 0;

	while(1) {
		// Que variable debo mostrar
		xQueuePeek(queue_display1, &change, portMAX_DELAY);
		xQueuePeek(queue_display2, &val, portMAX_DELAY);
		xQueuePeek(queue_lux, &lux, portMAX_DELAY);
		if(change == 0){
			show = val;
		}
		else{
			show = lux;
		}

		wrapper_display_off();
		wrapper_display_write((uint8_t)(show / 10));
		wrapper_display_on(COM_1);
		vTaskDelay(10);
		wrapper_display_off();
		wrapper_display_write((uint8_t)(show % 10));
		wrapper_display_on(COM_2);
		vTaskDelay(10);
	}
}

void task_PWM(void *params) {
	// Valores de ADC
	adc_data_t data = {0};
	uint8_t pwm = 0;

	while(1) {
		xQueueReceive(queue_adc, &data, portMAX_DELAY);
		pwm = (uint8_t)((data.ref_raw * 100) / 4095);
		wrapper_pwm_update(pwm);
		xQueueOverwrite(queue_pwm, &pwm);
	}
}

void ADC0_SEQA_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(kADC_ConvSeqAInterruptFlag == (kADC_ConvSeqAInterruptFlag & ADC_GetStatusFlags(ADC0))) {
		ADC_ClearStatusFlags(ADC0, kADC_ConvSeqAInterruptFlag);
		adc_result_info_t temp_info, ref_info;
		ADC_GetChannelConversionResult(ADC0, REF_POT_CH, &ref_info);
		ADC_GetChannelConversionResult(ADC0, LM35_CH, &temp_info);
		adc_data_t data = {
			.temp_raw = (uint16_t)temp_info.result,
			.ref_raw = (uint16_t)ref_info.result
		};
		xQueueOverwriteFromISR(queue_adc, &data, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}
