/*
 * dc_link.h
 *
 * DC-Link voltage-mode detection and short-circuit protection.
 */

#ifndef DC_LINK_DC_LINK_H_
#define DC_LINK_DC_LINK_H_

#include "main.h"
#include <stdint.h>

#define BATTERY_VOLTAGE_24V 24.0f
#define BATTERY_VOLTAGE_48V 48.0f
#define DEFAULT_BATTERY_VOLTAGE BATTERY_VOLTAGE_24V
#define BATTERY_AUTO_SWITCH_VOLTAGE 36.0f
#define BATTERY_24V_MIN_VOLTAGE 21.0f
#define BATTERY_24V_MAX_VOLTAGE 30.0f
#define BATTERY_48V_MIN_VOLTAGE 42.0f
#define BATTERY_48V_MAX_VOLTAGE 60.0f

class dc_link
{
private:
	enum VoltageMode : uint8_t {
		MODE_24V = 0,
		MODE_48V = 1
	};

	VoltageMode voltage_mode = MODE_24V;
	float nominal_voltage = BATTERY_VOLTAGE_24V;

public:
	dc_link();
	virtual ~dc_link();

	void Start(float measured_input_voltage);
	void Stop();

	static float AdcToVoltage(uint16_t adc);
	float ReadFastVoltage() const;
	float GetNominalVoltage() const;
	float GetLowVoltageLimit() const;
	float GetHighVoltageLimit() const;
	uint8_t Is24VMode() const;
};

extern dc_link *pDcLink;

#endif /* DC_LINK_DC_LINK_H_ */
