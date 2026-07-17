/*
 * fnd595.h
 *
 * 74HC595 7-segment FND driver
 */

#ifndef FND595_FND595_H_
#define FND595_FND595_H_

#include "main.h"
#include <stdint.h>

#ifndef FND595_SDAT_GPIO_Port
#define FND595_SDAT_GPIO_Port  pFND_SDAT_GPIO_Port
#define FND595_SDAT_Pin        pFND_SDAT_Pin
#endif

#ifndef FND595_SCLK_GPIO_Port
#define FND595_SCLK_GPIO_Port  pFND_SCLK_GPIO_Port
#define FND595_SCLK_Pin        pFND_SCLK_Pin
#endif

#ifndef FND595_LATCH_GPIO_Port
#define FND595_LATCH_GPIO_Port pFND_LATCH_GPIO_Port
#define FND595_LATCH_Pin       pFND_LATCH_Pin
#endif

class FND595
{
private:
	uint8_t last_pattern = 0;
	uint8_t active_high = 1;
	uint8_t msb_first = 1;
	uint16_t pulse_delay_us = 1000;
	uint8_t segment_map[8] = {0x40, 0x20, 0x01, 0x80, 0x04, 0x08, 0x10, 0x02};

	void write_data(uint8_t value);
	void pulse_clock();
	void pulse_latch();
	void shift_byte(uint8_t value);
	uint8_t map_segments(uint8_t pattern) const;
	uint8_t apply_polarity(uint8_t pattern) const;
	void wait_ms(uint16_t ms);

public:
	enum Segment : uint8_t {
		SEG_A  = 0x01,
		SEG_B  = 0x02,
		SEG_C  = 0x04,
		SEG_D  = 0x08,
		SEG_E  = 0x10,
		SEG_F  = 0x20,
		SEG_G  = 0x40,
		SEG_DP = 0x80
	};

	FND595();
	virtual ~FND595();

	void Init();
	void Clear();
	void WriteRaw(uint8_t pattern);
	void PrintDigit(uint8_t digit, uint8_t dot = 0);
	void PrintHex(uint8_t value, uint8_t dot = 0);
	void PrintMinus(uint8_t dot = 0);
	void SetDecimalPoint(uint8_t on);
	void SetActiveHigh(uint8_t on);
	void SetMsbFirst(uint8_t on);
	void SetPulseDelayUs(uint16_t us);
	void SetSegmentMap(const uint8_t map[8]);
	void PrintSegment(uint8_t segment);
	void TestSegments(uint16_t delay_ms = 10000);
	void PrintOutputBit(uint8_t bit);
	void TestOutputBits(uint16_t delay_ms = 10000);
	void TestPins(uint16_t count = 10, uint16_t delay_ms = 100);
	uint8_t EncodeHex(uint8_t value, uint8_t dot = 0) const;
	uint8_t GetLastPattern() const;
};

extern FND595 *pFND595;

#endif /* FND595_FND595_H_ */
