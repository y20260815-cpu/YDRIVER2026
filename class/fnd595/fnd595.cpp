/*
 * fnd595.cpp
 *
 * 74HC595 7-segment FND driver
 */

#include "extern.h"
#include "fnd595.h"
#include "iwdg.h"

FND595 *pFND595;

FND595::FND595()
{
	Init();
	Clear();
}

FND595::~FND595()
{
}

void FND595::Init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(FND595_SDAT_GPIO_Port, FND595_SDAT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FND595_SCLK_GPIO_Port, FND595_SCLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FND595_LATCH_GPIO_Port, FND595_LATCH_Pin, GPIO_PIN_RESET);

	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_InitStruct.Pin = FND595_SDAT_Pin;
	HAL_GPIO_Init(FND595_SDAT_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = FND595_SCLK_Pin;
	HAL_GPIO_Init(FND595_SCLK_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = FND595_LATCH_Pin;
	HAL_GPIO_Init(FND595_LATCH_GPIO_Port, &GPIO_InitStruct);
}

void FND595::write_data(uint8_t value)
{
	HAL_GPIO_WritePin(FND595_SDAT_GPIO_Port, FND595_SDAT_Pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void FND595::pulse_clock()
{
	HAL_GPIO_WritePin(FND595_SCLK_GPIO_Port, FND595_SCLK_Pin, GPIO_PIN_SET);
	if(pulse_delay_us) delay_us(pulse_delay_us);
	HAL_GPIO_WritePin(FND595_SCLK_GPIO_Port, FND595_SCLK_Pin, GPIO_PIN_RESET);
	if(pulse_delay_us) delay_us(pulse_delay_us);
}

void FND595::pulse_latch()
{
	HAL_GPIO_WritePin(FND595_LATCH_GPIO_Port, FND595_LATCH_Pin, GPIO_PIN_SET);
	if(pulse_delay_us) delay_us(pulse_delay_us);
	HAL_GPIO_WritePin(FND595_LATCH_GPIO_Port, FND595_LATCH_Pin, GPIO_PIN_RESET);
	if(pulse_delay_us) delay_us(pulse_delay_us);
}

uint8_t FND595::apply_polarity(uint8_t pattern) const
{
	return active_high ? pattern : (uint8_t)~pattern;
}

uint8_t FND595::map_segments(uint8_t pattern) const
{
	uint8_t mapped = 0;
	for(uint8_t bit = 0; bit < 8; bit++) {
		if(pattern & (uint8_t)(1U << bit)) {
			mapped |= segment_map[bit];
		}
	}
	return mapped;
}

void FND595::shift_byte(uint8_t value)
{
	if(msb_first) {
		for(uint8_t mask = 0x80; mask != 0; mask >>= 1) {
			write_data(value & mask);
			pulse_clock();
		}
	}
	else {
		for(uint8_t mask = 0x01; mask != 0; mask <<= 1) {
			write_data(value & mask);
			pulse_clock();
		}
	}
}

void FND595::wait_ms(uint16_t ms)
{
	while(ms >= 100) {
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(100);
		ms -= 100;
	}

	if(ms > 0) {
		HAL_IWDG_Refresh(&hiwdg);
		HAL_Delay(ms);
	}
}

void FND595::WriteRaw(uint8_t pattern)
{
	if(decimal_point_forced) pattern |= SEG_DP;
	// When not forced, preserve the DP bit supplied by PrintHex/PrintDigit.
	// Clearing it here made their dot argument ineffective.
	last_pattern = pattern;
	HAL_GPIO_WritePin(FND595_LATCH_GPIO_Port, FND595_LATCH_Pin, GPIO_PIN_RESET);
	shift_byte(apply_polarity(map_segments(pattern)));
	pulse_latch();
}

void FND595::Clear()
{
	WriteRaw(0x00);
}

uint8_t FND595::EncodeHex(uint8_t value, uint8_t dot) const
{
	static const uint8_t table[16] = {
		SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,
		SEG_B | SEG_C,
		SEG_A | SEG_B | SEG_D | SEG_E | SEG_G,
		SEG_A | SEG_B | SEG_C | SEG_D | SEG_G,
		SEG_B | SEG_C | SEG_F | SEG_G,
		SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,
		SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
		SEG_A | SEG_B | SEG_C,
		SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
		SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G,
		SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,
		SEG_C | SEG_D | SEG_E | SEG_F | SEG_G,
		SEG_A | SEG_D | SEG_E | SEG_F,
		SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,
		SEG_A | SEG_D | SEG_E | SEG_F | SEG_G,
		SEG_A | SEG_E | SEG_F | SEG_G
	};

	uint8_t pattern = table[value & 0x0F];
	if(dot) pattern |= SEG_DP;
	return pattern;
}

void FND595::PrintDigit(uint8_t digit, uint8_t dot)
{
	if(digit > 9) {
		Clear();
		return;
	}
	WriteRaw(EncodeHex(digit, dot));
}

void FND595::PrintHex(uint8_t value, uint8_t dot)
{
	WriteRaw(EncodeHex(value, dot));
}

void FND595::PrintMinus(uint8_t dot)
{
	uint8_t pattern = SEG_G;
	if(dot) pattern |= SEG_DP;
	WriteRaw(pattern);
}

void FND595::SetDecimalPoint(uint8_t on)
{
	decimal_point_forced = on ? 1 : 0;
	uint8_t pattern = last_pattern;
	if(on) pattern |= SEG_DP;
	else pattern &= (uint8_t)~SEG_DP;
	WriteRaw(pattern);
}

void FND595::SetActiveHigh(uint8_t on)
{
	active_high = on ? 1 : 0;
	WriteRaw(last_pattern);
}

void FND595::SetMsbFirst(uint8_t on)
{
	msb_first = on ? 1 : 0;
	WriteRaw(last_pattern);
}

void FND595::SetPulseDelayUs(uint16_t us)
{
	pulse_delay_us = us;
}

void FND595::SetSegmentMap(const uint8_t map[8])
{
	for(uint8_t i = 0; i < 8; i++) {
		segment_map[i] = map[i];
	}
	WriteRaw(last_pattern);
}

void FND595::PrintSegment(uint8_t segment)
{
	if(segment >= 8) {
		Clear();
		return;
	}
	WriteRaw((uint8_t)(1U << segment));
}

void FND595::PrintOutputBit(uint8_t bit)
{
	if(bit >= 8) {
		Clear();
		return;
	}

	last_pattern = 0;
	HAL_GPIO_WritePin(FND595_LATCH_GPIO_Port, FND595_LATCH_Pin, GPIO_PIN_RESET);
	shift_byte(apply_polarity((uint8_t)(1U << bit)));
	pulse_latch();
}

void FND595::TestSegments(uint16_t delay_ms)
{
	static const char *name[8] = {"A", "B", "C", "D", "E", "F", "G", "DP"};

	for(uint8_t segment = 0; segment < 8; segment++) {
		printf("[FND595] Test segment %s\r\n", name[segment]);
		PrintSegment(segment);
		wait_ms(delay_ms);
	}
	printf("[FND595] Test clear\r\n");
	Clear();
}

void FND595::TestOutputBits(uint16_t delay_ms)
{
	static const char *name[8] = {"QA", "QB", "QC", "QD", "QE", "QF", "QG", "QH"};

	for(uint8_t bit = 0; bit < 8; bit++) {
		printf("[FND595] Test output %s\r\n", name[bit]);
		PrintOutputBit(bit);
		wait_ms(delay_ms);
	}
	printf("[FND595] Test clear\r\n");
	Clear();
}

void FND595::TestPins(uint16_t count, uint16_t delay_ms)
{
	for(uint16_t i = 0; i < count; i++) {
		HAL_GPIO_TogglePin(FND595_SDAT_GPIO_Port, FND595_SDAT_Pin);
		HAL_GPIO_TogglePin(FND595_SCLK_GPIO_Port, FND595_SCLK_Pin);
		HAL_GPIO_TogglePin(FND595_LATCH_GPIO_Port, FND595_LATCH_Pin);
		HAL_Delay(delay_ms);
	}

	HAL_GPIO_WritePin(FND595_SDAT_GPIO_Port, FND595_SDAT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FND595_SCLK_GPIO_Port, FND595_SCLK_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FND595_LATCH_GPIO_Port, FND595_LATCH_Pin, GPIO_PIN_RESET);
}

uint8_t FND595::GetLastPattern() const
{
	return last_pattern;
}
