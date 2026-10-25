#ifndef SYSTEM_SETUP_H_
#define SYSTEM_SETUP_H_

#define DM_DRIVER 1
#define FND_DRIVER 2
#define PCB_TYPE DM_DRIVER

#if PCB_TYPE == DM_DRIVER
#define PCB_HAS_FND 0
#elif PCB_TYPE == FND_DRIVER
#define PCB_HAS_FND 1
#else
#error "Unsupported PCB_TYPE"
#endif

#define SYSTEM_MODEL_NAME "STM32_SB100D_BJ"
//#define SYSTEM_MODEL_NAME "STM32_DD100D_BJ"

#ifdef __cplusplus
#include "../../class/STM32_SB100D_BJ/STM32_SB100D_BJ.h"

constexpr bool SystemModelNameMatches(const char *name, const char *expected)
{
	return *name == *expected
			&& (*name == '\0' || SystemModelNameMatches(name + 1, expected + 1));
}

static_assert(SystemModelNameMatches(SYSTEM_MODEL_NAME, "STM32_SB100D_BJ"),
		"Unsupported SYSTEM_MODEL_NAME: add a model class and selection first");
using SystemModel = STM32_SB100D_BJ;

static_assert(SystemModel::CAN_INPUT_ONLY, "Only CAN-input models are supported");
static_assert(SystemModel::ACCEL_MS >= 1000U && SystemModel::ACCEL_MS <= 10000U
		&& SystemModel::ACCEL_MS % 10U == 0U, "Invalid model acceleration time");
static_assert(SystemModel::DECEL_MS >= 500U && SystemModel::DECEL_MS <= 10000U,
		"Invalid model deceleration time");
static_assert(SystemModel::BRAKE_DELAY_MS >= 100U && SystemModel::BRAKE_DELAY_MS <= 2000U
		&& SystemModel::BRAKE_DELAY_MS % 10U == 0U, "Invalid model brake delay");
static_assert(SystemModel::PID.Kp >= 0.0f && SystemModel::PID.Ki >= 0.0f
		&& SystemModel::PID.Kd == 0.0f, "Only PI control is implemented");
static_assert(SystemModel::NOMINAL_VOLTAGE == 24.0f
		&& SystemModel::LOW_VOLTAGE_LIMIT > 0.0f
		&& SystemModel::LOW_VOLTAGE_LIMIT < SystemModel::NOMINAL_VOLTAGE
		&& SystemModel::HIGH_VOLTAGE_LIMIT > SystemModel::NOMINAL_VOLTAGE
		&& SystemModel::HIGH_VOLTAGE_LIMIT <= 30.0f, "Invalid 24V model protection limits");
static_assert(SystemModel::CURRENT_LIMIT_A > 0.0f && SystemModel::CURRENT_LIMIT_A <= 120.0f,
		"Model current limit requires hardware validation above 120A");
static_assert(SystemModel::MOTOR1_REVERSED <= 1U && SystemModel::MOTOR2_REVERSED <= 1U,
		"Invalid model motor polarity");
#endif
#endif
