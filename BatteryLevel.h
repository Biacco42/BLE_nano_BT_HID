class BatteryLevel {

public:
	static const float BATTERY_MAX = 2.8;
	static const float REFERNECE = 1.2;
	static const float PRESCALE = 3;
	static const float BATTERY_LOW = 2.0;

	static uint8_t readBatteryPercentage(const float voltage) {
		uint16_t percentage = (voltage - BATTERY_LOW) / (BATTERY_MAX - BATTERY_LOW) * 100;
		if (percentage > 100) percentage = 100;
		return percentage;
	}

	static float readBatteryVoltage() {
		NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;

		// Use internal 1.2V reference for batteryInput
		//  1/3 pre-scaled input and 1.2V internal band gap reference
		// ref. mbed-src/targets/hal/TARGET_NORDIC/TARGET_MCU_NRF51822/analogin_api.c
		NRF_ADC->CONFIG =
			(ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
			// Use VDD 1/3 for input
			(ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
			// Use internal band gap for reference
			(ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
			(ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);

		// Start ADC
		NRF_ADC->TASKS_START = 1;
		while (((NRF_ADC->BUSY & ADC_BUSY_BUSY_Msk) >> ADC_BUSY_BUSY_Pos) == ADC_BUSY_BUSY_Busy) {
			// busy loop
		}

		// Read ADC result
		uint16_t raw10bit = static_cast<uint16_t>(NRF_ADC->RESULT);

		NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;

		float ratio = raw10bit / static_cast<float>(1<<10);

		float batteryVoltage = ratio * (REFERNECE * PRESCALE);
		return batteryVoltage;
	}
};
