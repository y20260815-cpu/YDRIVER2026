/*
 * dc_link.cpp
 *
 * DC-Link voltage-mode detection and short-circuit protection.
 */

#include "extern.h"
#include "dc_link.h"

dc_link *pDcLink;

dc_link::dc_link()
{
}

dc_link::~dc_link()
{
}

float dc_link::AdcToVoltage(uint16_t adc)
{
	return (float)adc * 3.3f / 4095.0f * 52.0f;
}

float dc_link::ReadFastVoltage() const
{
	return AdcToVoltage(adc_buf[7]);
}

void dc_link::Start(float measured_input_voltage)
{
	voltage_mode = (measured_input_voltage >= BATTERY_AUTO_SWITCH_VOLTAGE)
			? MODE_48V : MODE_24V;
	nominal_voltage = (voltage_mode == MODE_48V)
			? BATTERY_VOLTAGE_48V : BATTERY_VOLTAGE_24V;
}

void dc_link::Stop()
{
}

float dc_link::GetNominalVoltage() const
{
	return nominal_voltage;
}

float dc_link::GetLowVoltageLimit() const
{
	return (voltage_mode == MODE_48V)
			? BATTERY_48V_MIN_VOLTAGE : BATTERY_24V_MIN_VOLTAGE;
}

float dc_link::GetHighVoltageLimit() const
{
	return (voltage_mode == MODE_48V)
			? BATTERY_48V_MAX_VOLTAGE : BATTERY_24V_MAX_VOLTAGE;
}

uint8_t dc_link::Is24VMode() const
{
	return voltage_mode == MODE_24V;
}
