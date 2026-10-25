/*
 * dc_link.cpp
 *
 * DC-Link measurement and model voltage limits.
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
	nominal_voltage = SystemModel::NOMINAL_VOLTAGE;
	printf("MODEL %s: default voltage=%.1fV, measured DC-Link=%.1fV\r\n",
			SYSTEM_MODEL_NAME, SystemModel::NOMINAL_VOLTAGE, measured_input_voltage);
}

bool dc_link::SetNominalVoltage(uint8_t voltage)
{
	if(voltage != 24U && voltage != 48U) {
		printf("DC-Link: unsupported voltage command=%u\r\n", voltage);
		return false;
	}
	nominal_voltage = static_cast<float>(voltage);
	return true;
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
	return Is24VMode() ? SystemModel::LOW_VOLTAGE_LIMIT : 42.0f;
}

float dc_link::GetHighVoltageLimit() const
{
	return Is24VMode() ? SystemModel::HIGH_VOLTAGE_LIMIT : 60.0f;
}

uint8_t dc_link::Is24VMode() const
{
	return nominal_voltage == BATTERY_VOLTAGE_24V;
}
