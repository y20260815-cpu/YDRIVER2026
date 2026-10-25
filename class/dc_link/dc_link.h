/*
 * dc_link.h
 *
 * DC-Link measurement and model voltage limits.
 */

#ifndef DC_LINK_DC_LINK_H_
#define DC_LINK_DC_LINK_H_

#include "main.h"
#include <stdint.h>
#include "../../Core/Inc/system_setup.h"

#define BATTERY_VOLTAGE_24V 24.0f
#define DEFAULT_BATTERY_VOLTAGE SystemModel::NOMINAL_VOLTAGE

class dc_link
{
private:
	float nominal_voltage = SystemModel::NOMINAL_VOLTAGE;
public:
	dc_link();
	virtual ~dc_link();

	void Start(float measured_input_voltage);
	void Stop();
	bool SetNominalVoltage(uint8_t voltage);

	static float AdcToVoltage(uint16_t adc);
	float ReadFastVoltage() const;
	float GetNominalVoltage() const;
	float GetLowVoltageLimit() const;
	float GetHighVoltageLimit() const;
	uint8_t Is24VMode() const;
};

extern dc_link *pDcLink;

#endif /* DC_LINK_DC_LINK_H_ */
