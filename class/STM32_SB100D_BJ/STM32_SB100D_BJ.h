#ifndef STM32_SB100D_BJ_H_
#define STM32_SB100D_BJ_H_

#include <stdint.h>

typedef struct
{
	bool disable;
	float Kp;
	float Ki;
	float Kd;
} PID_CONTROL_CONFIG;

class STM32_SB100D_BJ
{
public:
	static constexpr bool CAN_INPUT_ONLY = true;
	static constexpr uint16_t ACCEL_MS = 2000U;
	static constexpr uint16_t DECEL_MS = 3000U;
	static constexpr uint16_t BRAKE_DELAY_MS = 300U;
	static constexpr PID_CONTROL_CONFIG PID = {
		true, // true disables PID regardless of the CAN command.
		0.8f,  // Kp
		0.1f,  // Ki
		0.0f   // Kd (PI only)
	};
	static constexpr float NOMINAL_VOLTAGE = 24.0f;
	static constexpr float LOW_VOLTAGE_LIMIT = 21.0f;
	static constexpr float HIGH_VOLTAGE_LIMIT = 30.0f;
	static constexpr float CURRENT_LIMIT_A = 120.0f;
	static constexpr uint8_t MOTOR1_REVERSED = 0U;
	static constexpr uint8_t MOTOR2_REVERSED = 0U;
};

#endif
