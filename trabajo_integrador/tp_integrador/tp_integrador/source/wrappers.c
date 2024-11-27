#include "wrappers.h"

static uint32_t pwm_led_event = 0;

void wrapper_adc_init(void) {
    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetFixedPinSelect(SWM0, kSWM_ADC_CHN0, true);
    SWM_SetFixedPinSelect(SWM0, kSWM_ADC_CHN8, true);
    CLOCK_DisableClock(kCLOCK_Swm);
    CLOCK_Select(kADC_Clk_From_Fro);
    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 1);

    POWER_DisablePD(kPDRUNCFG_PD_ADC0);

	uint32_t frequency = CLOCK_GetFreq(kCLOCK_Fro) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);
	ADC_DoSelfCalibration(ADC0, frequency);
	adc_config_t adc_config;
	ADC_GetDefaultConfig(&adc_config);
	ADC_Init(ADC0, &adc_config);
	adc_conv_seq_config_t adc_sequence = {
		.channelMask = 1 << LM35_CH | 1 << REF_POT_CH,
		.triggerMask = 0,
		.triggerPolarity = kADC_TriggerPolarityPositiveEdge,
		.enableSyncBypass = false,
		.interruptMode = kADC_InterruptForEachSequence
	};
	ADC_SetConvSeqAConfig(ADC0, &adc_sequence);
	ADC_EnableConvSeqA(ADC0, true);

    ADC_EnableInterrupts(ADC0, kADC_ConvSeqAInterruptEnable);
    NVIC_EnableIRQ(ADC0_SEQA_IRQn);
}

void wrapper_btn_init(void) {
	gpio_pin_config_t config = { kGPIO_DigitalInput };
	uint32_t pins[] = { USR_BTN, ISP_BTN, S1_BTN, S2_BTN };
	for(uint8_t i = 0; i < sizeof(pins) / sizeof(uint32_t); i++) {
		GPIO_PinInit(GPIO, 0, pins[i], &config);
	}
}

void wrapper_display_init(void) {
	gpio_pin_config_t config = { kGPIO_DigitalOutput, true };
	uint32_t pins[] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, COM_1, COM_2 };
	for(uint8_t i = 0; i < sizeof(pins) / sizeof(uint32_t); i++) {
		GPIO_PinInit(GPIO, 0, pins[i], &config);
		GPIO_PinWrite(GPIO, 0, pins[i], 1);
	}
}

void wrapper_display_write(uint8_t number) {
	uint8_t values[] = { ~0x3f, ~0x6, ~0x5b, ~0x4f, ~0x66, ~0x6d, ~0x7d, ~0x7, ~0x7f, ~0x6f };
	uint32_t pins[] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G };

	for(uint8_t i = 0; i < sizeof(pins) / sizeof(uint32_t); i++) {
		uint32_t val = (values[number] & (1 << i))? 1 : 0;
		GPIO_PinWrite(GPIO, 0, pins[i], val);
	}
}

void wrapper_pwm_init(void) {
    CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetMovablePinSelect(SWM0, kSWM_SCT_OUT4, kSWM_PortPin_P0_0 + LED);
    CLOCK_DisableClock(kCLOCK_Swm);

    uint32_t sctimer_clock = CLOCK_GetFreq(kCLOCK_Fro);
    sctimer_config_t sctimer_config;
    SCTIMER_GetDefaultConfig(&sctimer_config);
    SCTIMER_Init(SCT0, &sctimer_config);

    sctimer_pwm_signal_param_t pwm_config = {
		.output = kSCTIMER_Out_4,
		.level = kSCTIMER_HighTrue,
		.dutyCyclePercent = 0
    };

    SCTIMER_SetupPwm(
		SCT0,
		&pwm_config,
		kSCTIMER_CenterAlignedPwm,
		1000,
		sctimer_clock,
		&pwm_led_event
	);
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);
}

void wrapper_pwm_update(int16_t duty) {
	if(duty < 0) {
		duty = 0;
	}
	else if(duty > 100) {
		duty = 100;
	}
	SCTIMER_UpdatePwmDutycycle(SCT0, kSCTIMER_Out_4, duty, pwm_led_event);
}

void wrapper_i2c_init(void) {
	CLOCK_Select(kI2C1_Clk_From_MainClk);
	CLOCK_EnableClock(kCLOCK_Swm);
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SDA, kSWM_PortPin_P0_27);
    SWM_SetMovablePinSelect(SWM0, kSWM_I2C1_SCL, kSWM_PortPin_P0_26);
    CLOCK_DisableClock(kCLOCK_Swm);

    i2c_master_config_t config;
    I2C_MasterGetDefaultConfig(&config);
    config.baudRate_Bps = 400000;
    I2C_MasterInit(I2C1_BASE, &config, SystemCoreClock);
}

void wrapper_bh1750_init(void) {
	uint8_t cmd[] = { 0x01, 0x10 };
	I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write);
	I2C_MasterWriteBlocking(I2C1, &cmd[0], 1, kI2C_TransferDefaultFlag);
	I2C_MasterStop(I2C1);
	I2C_MasterStart(I2C1, BH1750_ADDR, kI2C_Write);
	I2C_MasterWriteBlocking(I2C1, &cmd[1], 1, kI2C_TransferDefaultFlag);
	I2C_MasterStop(I2C1);
}

float wrapper_bh1750_read(void) {
	uint8_t res[2] = {0};
	I2C_MasterRepeatedStart(I2C1, BH1750_ADDR, kI2C_Read);
	I2C_MasterReadBlocking(I2C1, res, 2, kI2C_TransferDefaultFlag);
	I2C_MasterStop(I2C1);
	return ((res[0] << 8) + res[1]) / 1.2;
}