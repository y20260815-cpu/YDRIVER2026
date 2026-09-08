/*
 * shunt_oc.h
 *
 * Full-scale over-current input protection on pAN3_OC (PA2 / ADC1_IN2).
 */

#ifndef SHUNT_OC_SHUNT_OC_H_
#define SHUNT_OC_SHUNT_OC_H_

#include "main.h"
#include <stdint.h>

// PA2 external shunt input is not reliable on the current hardware.
// Keep the implementation available for future boards, but disable its
// analog-watchdog trip path. WCS1600 software over-current protection remains.
#define SHUNT_OC_ENABLE 0

#define SHUNT_OC_ADC_INDEX 2U
#define SHUNT_OC_ADC_CHANNEL ADC_CHANNEL_2
#define SHUNT_OC_THRESHOLD_24V 2000U
#define SHUNT_OC_THRESHOLD_48V 2500U
#define SHUNT_OC_DC_LINK_THRESHOLD_24V 18.0f
#define SHUNT_OC_DC_LINK_THRESHOLD_48V 36.0f

class shunt_oc
{
private:
	volatile uint8_t enabled = 0;
	volatile uint8_t tripped = 0;
	volatile uint16_t trip_adc = 0;
	volatile float trip_voltage = 0.0f;
	uint16_t active_threshold = SHUNT_OC_THRESHOLD_24V;
	float active_voltage_threshold = SHUNT_OC_DC_LINK_THRESHOLD_24V;

	void ConfigureAnalogWatchdog();
	void EmergencyShutdown();

public:
	shunt_oc();
	virtual ~shunt_oc();

	void Start();
	void Stop();
	void HandleAnalogWatchdog(ADC_HandleTypeDef *hadc);

	uint8_t IsTripped() const;
	uint16_t GetTripAdc() const;
	uint16_t GetThresholdAdc() const;
	float GetTripVoltage() const;
	float GetVoltageThreshold() const;
	uint16_t GetRawAdc() const;
	uint16_t GetPan4RawAdc() const;
};

extern shunt_oc *pShuntOc;

#endif /* SHUNT_OC_SHUNT_OC_H_ */
