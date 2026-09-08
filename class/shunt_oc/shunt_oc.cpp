/*
 * shunt_oc.cpp
 *
 * Full-scale over-current input protection on pAN3_OC (PA2 / ADC1_IN2).
 */

#include "extern.h"
#include "shunt_oc.h"

shunt_oc *pShuntOc;

shunt_oc::shunt_oc()
{
}

shunt_oc::~shunt_oc()
{
}

void shunt_oc::Start()
{
	tripped = 0;
	trip_adc = 0;
	trip_voltage = 0.0f;
	active_threshold = (pDcLink != 0 && pDcLink->Is24VMode())
			? SHUNT_OC_THRESHOLD_24V : SHUNT_OC_THRESHOLD_48V;
	active_voltage_threshold = (pDcLink != 0 && pDcLink->Is24VMode())
			? SHUNT_OC_DC_LINK_THRESHOLD_24V
			: SHUNT_OC_DC_LINK_THRESHOLD_48V;
	enabled = 1;
	ConfigureAnalogWatchdog();
	if(HAL_ADC_Start(&hadc2) != HAL_OK)
		Error_Handler();
}

void shunt_oc::Stop()
{
	enabled = 0;
	__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_AWD);
	HAL_ADC_Stop(&hadc2);
}

void shunt_oc::ConfigureAnalogWatchdog()
{
	// ADC2 converts PA2 only, so its DR remains a pAN3_OC value.
	__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_AWD);
	WRITE_REG(hadc2.Instance->LTR, 0U);
	WRITE_REG(hadc2.Instance->HTR, active_threshold);
	MODIFY_REG(hadc2.Instance->CR1,
			ADC_CR1_AWDSGL | ADC_CR1_JAWDEN | ADC_CR1_AWDEN | ADC_CR1_AWDCH,
			ADC_CR1_AWDSGL | ADC_CR1_AWDEN
					| (SHUNT_OC_ADC_CHANNEL & ADC_CR1_AWDCH));
	__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_AWD);
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_AWD);
}

void shunt_oc::HandleAnalogWatchdog(ADC_HandleTypeDef *hadc)
{
	if(hadc == 0 || hadc->Instance != ADC2 || !enabled || tripped)
		return;

	// Confirm a real DC-Link short using two independent symptoms:
	// 1) PA2 pAN3_OC exceeds its voltage-mode threshold (this AWD interrupt).
	// 2) DC-Link voltage simultaneously collapses below its mode threshold.
	// A PWM switching spike with a healthy DC-Link is ignored.
	uint16_t oc_adc = (uint16_t)(hadc->Instance->DR & 0x0FFFU);
	float dc_link_voltage = (pDcLink != 0) ? pDcLink->ReadFastVoltage() : 0.0f;
	if(dc_link_voltage > active_voltage_threshold)
		return;

	trip_adc = oc_adc;
	trip_voltage = dc_link_voltage;
	tripped = 1;
	enabled = 0;
	__HAL_ADC_DISABLE_IT(hadc, ADC_IT_AWD);
	EmergencyShutdown();
}

void shunt_oc::EmergencyShutdown()
{
	// Remove bridge drive first, then open both DC-Link relays:
	// MC1 (pRY1) = DC-Link main relay.
	// MC2 (pRY2) = DC-Link precharge relay.
	// The GPIO writes are back-to-back because the relays use different ports.
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_RESET);
	htim2.Instance->CCR1 = 0;
	htim2.Instance->CCR2 = 1024;
	htim2.Instance->CCR3 = 0;
	htim2.Instance->CCR4 = 1024;
	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = 1024;
	htim3.Instance->CCR3 = 0;
	htim3.Instance->CCR4 = 1024;
	HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_RESET);
}

uint8_t shunt_oc::IsTripped() const
{
	return tripped;
}

uint16_t shunt_oc::GetTripAdc() const
{
	return trip_adc;
}

uint16_t shunt_oc::GetThresholdAdc() const
{
	return active_threshold;
}

float shunt_oc::GetTripVoltage() const
{
	return trip_voltage;
}

float shunt_oc::GetVoltageThreshold() const
{
	return active_voltage_threshold;
}

uint16_t shunt_oc::GetRawAdc() const
{
	return adc_buf[SHUNT_OC_ADC_INDEX];
}

uint16_t shunt_oc::GetPan4RawAdc() const
{
	// pAN4_OC = PA3 / ADC1_IN3 / DMA scan index 3.
	return adc_buf[3];
}

extern "C" void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
	if(pShuntOc != 0)
		pShuntOc->HandleAnalogWatchdog(hadc);
}
