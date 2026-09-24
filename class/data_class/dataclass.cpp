/*
 * dataclass.cpp
 *
 *  Created on: Jul 19, 2024
 *      Author: thpark
 */
#include "extern.h"
#include "dataclass.h"
#include "math.h"
#include <stdio.h>

//#define B_OFFSET 300 //UMJI _Thottle

data_class *pDataClass;

data_class::data_class()
{
	// TODO Auto-generated constructor stub
	memset(&gMAIN,0,sizeof(gMAIN));
	memset(&sysFlag,0,sizeof(sysFlag));
	memset(&board_io,0,sizeof(MY_IO));
	memset(&ex_board_io,0,sizeof(MY_IO));
	memset(&inputRaw,0,sizeof(POSITION_ANGLE));
	memset(&batt,0,sizeof(MY_BATTERY));
	memset(&error_code,0,sizeof(ERROR_CODE_STATE));
	memset(ladcValue,0,sizeof(ladcValue));

	gMAIN.flg_state.trot_zero=1;
	relayAllOFF();
}

data_class::~data_class()
{
	// TODO Auto-generated destructor stub
}

void data_class::power_on(){
	PWR_ON_Flg=1;
	ClearMotorCommandQueue();
	over_current_10ms_cnt = 0;
	over_current_retry_wait_10ms_cnt = 0;
	over_current_retry_count = 0;
	over_current_protect = 0;
	fet_temp_over_cnt = 0;
	fet_temp_protect = 0;
	currentPWM1 = 0.0f;
	currentPWM2 = 0.0f;
	fm2000_dir1 = 0;
	fm2000_dir2 = 0;
	fm2000_dir_change_wait1 = 0;
	fm2000_dir_change_wait2 = 0;
	memset(&inputRaw,0,sizeof(POSITION_ANGLE));
	// DC-Link power-on sequence:
	// 1) MC2 precharge relay ON to charge the DC-Link capacitor.
	// 2) After the precharge delay, MC1 main relay ON.
	gMAIN.relay.MC2=1;
	HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);

	gMAIN.relay.MC1=1;
	HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	// Arm DC-Link protection only after MC1 switching transients have settled.
	// With SD still LOW, the motor bridge cannot draw short-circuit current.
	float input_voltage = dc_link::AdcToVoltage(adc_buf[7]);
	pDcLink->Start(input_voltage);
	float detected_voltage = pDcLink->GetNominalVoltage();
	batt.use_battery_voltage = detected_voltage;
	batt.motor_spec_voltage = detected_voltage;
	// Seed BEMF offsets from a 100-sample average while the bridge is still
	// disabled; the one-shot runtime capture can start from a noisy sample.
	calibrate_wcs1600_zero();
	pPWM->Calibrate_BEMF_Offset();

	// Arm PA2 full-scale over-current protection only on boards that have a
	// calibrated external shunt circuit. WCS1600 protection is independent.
#if SHUNT_OC_ENABLE
	pShuntOc->Start();
#endif
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_SET);
	// The electromagnetic brake is spring-applied: no relay power means DN.
	// Start in the safe applied state. A valid drive command first builds
	// limited holding torque, then releases and settles the brake in PWM16.
	pPWM->brake_on_flag=1;
	HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);

	//motor1_polarity=sysConf.motor1_polarity;
	//motor2_polarity=sysConf.motor2_polarity;

	//gamsok_idx=sysConf.brake_rate;
	// Defaults before the first CAN frame: byte2 will override the EMB delay,
	// acceleration stays at the fixed constant.
	brake_delay=ELECTROMAGNETIC_BRAKE_DELAY_MS;
	can_accel_rate = ACCEL_RATE;
	can_accel_duration_10ms = ACCEL_DURATION_10MS_TICKS;
	can_decel_rpm_per_10ms = DECEL_RPM_PER_10MS;
	//set_foreward=sysConf.foreward/100.0f;
	//set_backward=set_foreward;
	//batt.motor_spec_rpm=MOTOR_MAX_RPM;
	// Normalize the PID to the measured speed range so Kp reads as
	// "correction PWM per full-scale error".
	pidCONF.rpm_to_pwm_scale=1.0f/CONTROL_TARGET_MAX_RPM;
	pidCONF.Kp=DEFAULT_PID_KP;


	//pPWM->pid_k
    //sysConf.motor1_polarity = 1;  // 0 → 1로 변경
    //sysConf.motor2_polarity = 1;  // 0 → 1로 변경
}

void data_class::power_off(){
	PWR_ON_Flg=0;
	// An intentional relay opening must not be reported as a DC-link short.
	if(pDcLink != 0) pDcLink->Stop();
#if SHUNT_OC_ENABLE
	if(pShuntOc != 0) pShuntOc->Stop();
#endif
	ClearMotorCommandQueue();
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_RESET);//FET ALL OFF
	// DC-Link power-off sequence: MC1 main and MC2 precharge relays OFF
	// back-to-back. They are on different GPIO ports, so this is the closest
	// software equivalent to simultaneous relay opening.
	gMAIN.relay.MC1=0;
	HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_RESET);//MC1 DC-Link main relay OFF
	gMAIN.relay.MC2=0;
	HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_RESET);//MC2 precharge relay OFF
	pPWM->brake_on_flag=1;
	HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);//EMB No Power
	HAL_Delay(500);
	relayAllOFF();
}

void data_class::relayAllOFF()
{
	// MC1 = DC-Link main relay, MC2 = DC-Link precharge relay.
	HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pRY5_GPIO_Port, pRY5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pRY6_GPIO_Port, pRY6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pWUP_GPIO_Port, pWUP_Pin, GPIO_PIN_RESET);//CSV1
	HAL_GPIO_WritePin(pWDN_GPIO_Port, pWDN_Pin, GPIO_PIN_RESET);//CSV2
}
void data_class::relay_control()
{
	// MC1 controls the DC-Link main path.
	if(gMAIN.relay.MC1) HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_RESET);

	// MC2 controls the DC-Link precharge path.
	if(gMAIN.relay.MC2) HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_RESET);
}

//motor
//----------
//sv1
//sv2
//--------------
//csv1
//csv2

void data_class::Lift_Cast_Stop(void){
	HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_RESET);//M
	HAL_GPIO_WritePin(pRY5_GPIO_Port, pRY5_Pin, GPIO_PIN_RESET);//SV1
	HAL_GPIO_WritePin(pRY6_GPIO_Port, pRY6_Pin, GPIO_PIN_RESET);//SV2
	HAL_GPIO_WritePin(pWUP_GPIO_Port, pWUP_Pin, GPIO_PIN_RESET);//CSV1
	HAL_GPIO_WritePin(pWDN_GPIO_Port, pWDN_Pin, GPIO_PIN_RESET);//CSV2
}

void data_class::Lift_UpDn(uint8_t cmd){
	HAL_GPIO_WritePin(pWUP_GPIO_Port, pWUP_Pin, GPIO_PIN_RESET);//CSV1
	HAL_GPIO_WritePin(pWDN_GPIO_Port, pWDN_Pin, GPIO_PIN_RESET);//CSV2
	switch(cmd){
		case 0:
			HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_RESET);//M
			HAL_GPIO_WritePin(pRY5_GPIO_Port, pRY5_Pin, GPIO_PIN_RESET);//SV1
			HAL_GPIO_WritePin(pRY6_GPIO_Port, pRY6_Pin, GPIO_PIN_RESET);//SV2
			break;
		case 1://up
			HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_SET);//M
			HAL_GPIO_WritePin(pRY5_GPIO_Port, pRY5_Pin, GPIO_PIN_SET);//SV1
			HAL_GPIO_WritePin(pRY6_GPIO_Port, pRY6_Pin, GPIO_PIN_RESET);//SV2
			break;
		case 2://dn
			HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_RESET);//M
			HAL_GPIO_WritePin(pRY5_GPIO_Port, pRY5_Pin, GPIO_PIN_SET);//SV1
			HAL_GPIO_WritePin(pRY6_GPIO_Port, pRY6_Pin, GPIO_PIN_SET);//SV2
			break;
	}
}

void data_class::Cast_UpDn(uint8_t cmd){
	HAL_GPIO_WritePin(pRY5_GPIO_Port, pRY5_Pin, GPIO_PIN_RESET);//SV1
	HAL_GPIO_WritePin(pRY6_GPIO_Port, pRY6_Pin, GPIO_PIN_RESET);//SV2
	switch(cmd){
		case 0:
			HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_RESET);//M
			HAL_GPIO_WritePin(pWUP_GPIO_Port, pWUP_Pin, GPIO_PIN_RESET);//CSV1
			HAL_GPIO_WritePin(pWDN_GPIO_Port, pWDN_Pin, GPIO_PIN_RESET);//CSV2
			break;
		case 1://up
			HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_SET);//M
			HAL_GPIO_WritePin(pWUP_GPIO_Port, pWUP_Pin, GPIO_PIN_SET);//CSV1
			HAL_GPIO_WritePin(pWDN_GPIO_Port, pWDN_Pin, GPIO_PIN_RESET);//CSV2
			break;
		case 2://dn
			HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_SET);//M
			HAL_GPIO_WritePin(pWUP_GPIO_Port, pWUP_Pin, GPIO_PIN_RESET);//CSV1
			HAL_GPIO_WritePin(pWDN_GPIO_Port, pWDN_Pin, GPIO_PIN_SET);//CSV2
			break;
	}
}

void data_class::lift_control(uint8_t Xlift, uint8_t Wlift)
{
	uint8_t en_lift_no=0;
	if(Xlift) en_lift_no=1;
	if(Wlift) en_lift_no=2;

	switch(en_lift_no){
		case 0:
			Lift_Cast_Stop();
			break;
		case 1:
			Lift_UpDn(Xlift);
			break;
		case 2:
			Cast_UpDn(Wlift);
			break;
	}
}


float data_class::LogRampStep(float target, float current, uint32_t elapsed_ms)
{
    float error = target - current;
    // Finish the final small step instead of leaving a residual PWM below
    // the ramp threshold.
    if (fabsf(error) < 0.01f)   return error;
    // 감속 판단: 목표 절대값이 현재 절대값보다 작으면 감속 (부호 무관)
    uint8_t is_decelerating = (fabsf(target) < fabsf(current));
    // 스텝 계산: 오차가 클 때는 큰 스텝, 작을 때는 작은 스텝
    float step;
    if(is_decelerating) {
        float decel_scale = 1.0f;
        float dc_link_voltage = pDcLink->ReadFastVoltage();
        float reference_voltage = (stop_vdc_reference > 1.0f)
                ? stop_vdc_reference : dc_link_voltage;
        float slow_voltage = reference_voltage * DECEL_VDC_SLOW_RATIO;
        float hold_voltage = reference_voltage * DECEL_VDC_HOLD_RATIO;

        if(dc_link_voltage >= hold_voltage) {
            decel_scale = DECEL_VDC_MIN_SCALE;
        }
        else if(dc_link_voltage > slow_voltage) {
            decel_scale = (hold_voltage - dc_link_voltage) /
                    (hold_voltage - slow_voltage);
            if(decel_scale < DECEL_VDC_MIN_SCALE)
                decel_scale = DECEL_VDC_MIN_SCALE;
        }

        step = (can_decel_rpm_per_10ms / CONTROL_TARGET_MAX_RPM) * decel_scale;
    }
	else {
		/*
		 * Time-based S-curve acceleration.  The inverse smoothstep recovers
		 * the current phase, then advances it by one 10 ms control tick.
		 * This remains continuous when a new CAN target arrives during a ramp
		 * and guarantees that 0 -> 100% takes can_accel_duration_10ms ticks.
		 */
		float current_abs = fabsf(current);
		if(current_abs > 1.0f) current_abs = 1.0f;
		float phase_lo = 0.0f;
		float phase_hi = 1.0f;
		for(uint8_t i = 0; i < 12; i++) {
			float phase_mid = (phase_lo + phase_hi) * 0.5f;
			float value_mid = phase_mid * phase_mid * (3.0f - 2.0f * phase_mid);
			if(value_mid < current_abs) phase_lo = phase_mid;
			else phase_hi = phase_mid;
		}
		float phase = (phase_lo + phase_hi) * 0.5f;
		uint16_t duration_ticks = can_accel_duration_10ms;
		if(duration_ticks < CAN_ACCEL_TIME_MIN_RAW * 10U)
			duration_ticks = CAN_ACCEL_TIME_MIN_RAW * 10U;
		float duration_ms = (float)duration_ticks * 10.0f;
		phase += (float)elapsed_ms / duration_ms;
		if(phase > 1.0f) phase = 1.0f;
		float next_abs = phase * phase * (3.0f - 2.0f * phase);
		step = next_abs - current_abs;
	}
    // 오차 부호에 맞게 스텝 부호 적용
    if (error < 0)
        step = -step;
    return step;
}

float data_class::PWM_UpdateRoutine(float targetPWM, float currentPWM, uint8_t throt_zero, uint32_t &last_update_ms, S_CURVE_STATE &curve)
{
	// stop_throttle is a state derived from the previous zero command.  It
	// must not force a newly received drive command back to zero; doing so
	// prevents CheckBrakeState() from ever seeing the release request and
	// leaves the spring-applied brake permanently engaged.
	(void)throt_zero;
    // 로그 기반으로 변화량(스텝) 계산
    uint32_t now_ms = control_millis;
    uint32_t elapsed_ms = (last_update_ms == 0U) ? 10U : (now_ms - last_update_ms);
    last_update_ms = now_ms;
    // Avoid a large jump after debugging, flash operations, or a long fault hold.
    if(elapsed_ms > 100U) elapsed_ms = 100U;
	uint8_t same_direction = (targetPWM >= 0.0f && currentPWM >= 0.0f)
			|| (targetPWM <= 0.0f && currentPWM <= 0.0f);
	uint8_t accelerating = same_direction
			&& (fabsf(targetPWM) > fabsf(currentPWM) + 0.0001f);
	if(accelerating) {
		if(!curve.active || fabsf(curve.target_pwm - targetPWM) > 0.001f) {
			curve.start_pwm = currentPWM;
			curve.target_pwm = targetPWM;
			curve.elapsed_ms = 0U;
			curve.active = 1U;
		}
		curve.elapsed_ms += elapsed_ms;
		float duration_ms = (float)can_accel_duration_10ms * 10.0f;
		float phase = (duration_ms > 0.0f)
				? (float)curve.elapsed_ms / duration_ms : 1.0f;
		if(phase >= 1.0f) {
			curve.active = 0U;
			return targetPWM;
		}
		float smooth = phase * phase * (3.0f - 2.0f * phase);
		return curve.start_pwm + (curve.target_pwm - curve.start_pwm) * smooth;
	}
	curve.active = 0U;
    float step = LogRampStep(targetPWM, currentPWM, elapsed_ms);
    // 오버슈팅 방지: 스텝이 남은 오차보다 크면 바로 목표값으로 설정
    if (fabsf(step) > fabsf(targetPWM - currentPWM))
        currentPWM = targetPWM;
    else
        currentPWM += step;
    return currentPWM;
}

float data_class::get_fet_temp_pwm_scale()
{
	int8_t fet_temp = (int8_t)batt.fet_temp;

	if(fet_temp_protect || fet_temp >= FET_TEMP_PWM_OFF) return 0.0f;
	if(fet_temp <= FET_TEMP_DERATE_START) return 1.0f;

	float scale = (float)(FET_TEMP_PWM_OFF - fet_temp) /
	              (float)(FET_TEMP_PWM_OFF - FET_TEMP_DERATE_START);
	if(scale < 0.0f) scale = 0.0f;
	if(scale > 1.0f) scale = 1.0f;
	return scale;
}


void data_class::stabilize_change(){
	if(state_change_flag_timeout){
		state_change_flag_timeout--;
		//printf("###### state_change_flag_timeout[%d]=====\r\n", state_change_flag_timeout);
		if(board_io.toggle.jenhujin !=0){
			state_change_flag_timeout=STABILIZE_DELAY_10MS_TICK;
		}
		inputRaw.source_pwm.f1=0.0f;
		inputRaw.source_pwm.f2=0.0f;
		inputRaw.limit_ratio.f1=0.0f;
		inputRaw.limit_ratio.f2=0.0f;
	}
}

float data_class::scale_value(int value, int min_val, int max_val)
{
    if (value <= min_val) return 0.0f;
    if (value >= max_val) return 1.0f;
    return (float)(value - min_val) / (float)(max_val - min_val);
}

float data_class::adc_to_pwm(uint16_t value)
{
	float pwm = (float)value / 4095.0f;
	if(pwm < 0.0f) pwm = 0.0f;
	if(pwm > 0.90f) pwm = 1.0f;
	return pwm;
}

float data_class::Calcu_motor_command_pwm(float requested_pwm, uint8_t &confirmed_dir, float &current_pwm, uint8_t &dir_change_wait_cnt, uint32_t &last_update_ms, S_CURVE_STATE &curve)
{
	uint8_t request_dir = 0;
	float target_pwm = requested_pwm;
	const float stop_threshold = 0.01f;
	// The PWM ramp and measured zero speed already provide the mechanical
	// stop interval. Keep only 100 ms of bridge settling time before applying
	// the opposite direction.
	const uint8_t direction_change_wait_10ms = 10;

	if(target_pwm > stop_threshold) {
		request_dir = 1;
	}
	else if(target_pwm < -stop_threshold) {
		request_dir = 2;
	}

	if(fet_temp_protect || fabsf(target_pwm) < stop_threshold) {
		request_dir = 0;
		target_pwm = 0.0f;
	}

	uint8_t current_dir = 0;
	if(current_pwm > stop_threshold) current_dir = 1;
	else if(current_pwm < -stop_threshold) current_dir = 2;

	if(request_dir != 0 && confirmed_dir != 0 && request_dir != confirmed_dir) {
		target_pwm = 0.0f;
		if(current_dir == 0) {
			if(dir_change_wait_cnt < direction_change_wait_10ms) dir_change_wait_cnt++;
			if(dir_change_wait_cnt >= direction_change_wait_10ms) {
				confirmed_dir = request_dir;
				target_pwm = requested_pwm;
				dir_change_wait_cnt = 0;
			}
		}
		else {
			dir_change_wait_cnt = 0;
		}
	}
	else if(request_dir != 0 && confirmed_dir == 0 && current_dir == 0) {
		confirmed_dir = request_dir;
		dir_change_wait_cnt = 0;
	}

	if(request_dir == 0 && current_dir == 0) {
		confirmed_dir = 0;
		dir_change_wait_cnt = 0;
	}

	// Never open the bridge abruptly when the volume reaches zero.
	// First ramp the drive PWM down; short braking takes over near zero.
	current_pwm = PWM_UpdateRoutine(target_pwm, current_pwm, sysFlag.stop_throttle, last_update_ms, curve);
	return current_pwm;
}

static uint8_t motor_command_class(float pwm)
{
	if(pwm > 0.01f) return 1;
	if(pwm < -0.01f) return 2;
	return 0;
}

void data_class::QueueMotorCommand(float pwm1, float pwm2)
{
	const float same_command_epsilon = 0.001f;

	if(pwm1 > 1.0f) pwm1 = 1.0f;
	if(pwm1 < -1.0f) pwm1 = -1.0f;
	if(pwm2 > 1.0f) pwm2 = 1.0f;
	if(pwm2 < -1.0f) pwm2 = -1.0f;

	// 비주기적으로 같은 명령이 반복 수신되어도 다시 큐에 넣지 않는다.
	if(last_received_command_valid
			&& fabsf(last_received_command_pwm1 - pwm1) <= same_command_epsilon
			&& fabsf(last_received_command_pwm2 - pwm2) <= same_command_epsilon) {
		return;
	}

	// 정상적으로는 10 ms 소비자가 매 주기 처리한다. 큐가 찬 경우에도
	// 최신 정지/방향 명령을 잃지 않도록 가장 오래된 항목을 제거한다.
	if(motor_command_count >= MOTOR_COMMAND_QUEUE_DEPTH) {
		motor_command_head = (motor_command_head + 1) % MOTOR_COMMAND_QUEUE_DEPTH;
		motor_command_count--;
	}

	MOTOR_COMMAND &command = motor_command_queue[motor_command_tail];
	command.pwm1 = pwm1;
	command.pwm2 = pwm2;
	command.sequence = ++motor_command_sequence;
	motor_command_tail = (motor_command_tail + 1) % MOTOR_COMMAND_QUEUE_DEPTH;
	motor_command_count++;
	last_received_command_pwm1 = pwm1;
	last_received_command_pwm2 = pwm2;
	last_received_command_valid = 1;
}

void data_class::ProcessMotorCommandQueue()
{
	const float stop_threshold = 0.01f;

	// Once a running motor receives a stop command, keep zero as the active
	// target throughout its ramp-down. A newer drive command, however, takes
	// over immediately from the current ramp value instead of waiting for
	// zero, so a re-applied throttle resumes from the present speed.
	if(stop_ramp_active_mask & 0x01) {
		if(fabsf(currentPWM1) <= stop_threshold) stop_ramp_active_mask &= ~0x01;
	}
	if(stop_ramp_active_mask & 0x02) {
		if(fabsf(currentPWM2) <= stop_threshold) stop_ramp_active_mask &= ~0x02;
	}
	if(stop_ramp_active_mask != 0) {
		if(motor_command_count == 0) return;
		const MOTOR_COMMAND &next = motor_command_queue[motor_command_head];
		// Opposite-direction requests remain safe to release here:
		// Calcu_motor_command_pwm() still forces a full stop plus settle
		// time before the confirmed direction may flip.
		if(fabsf(next.pwm1) <= stop_threshold
				&& fabsf(next.pwm2) <= stop_threshold) return;
		stop_ramp_active_mask = 0;
	}

	if(motor_command_count > 0) {
		const MOTOR_COMMAND &command = motor_command_queue[motor_command_head];
		uint8_t entering_stop = 0;
		uint8_t starting_drive =
				(fabsf(active_command_pwm1) <= stop_threshold
						&& fabsf(command.pwm1) > stop_threshold)
				|| (fabsf(active_command_pwm2) <= stop_threshold
						&& fabsf(command.pwm2) > stop_threshold);
		// A reference captured by the previous stop is no longer valid after
		// the supply voltage changes or a new drive cycle begins.
		if(starting_drive) stop_vdc_reference = 0.0f;
		if(fabsf(active_command_pwm1) > stop_threshold
				&& fabsf(command.pwm1) <= stop_threshold) {
			stop_ramp_active_mask |= 0x01;
			entering_stop = 1;
		}
		if(fabsf(active_command_pwm2) > stop_threshold
				&& fabsf(command.pwm2) <= stop_threshold) {
			stop_ramp_active_mask |= 0x02;
			entering_stop = 1;
		}
		if(entering_stop) stop_vdc_reference = pDcLink->ReadFastVoltage();
		active_command_pwm1 = command.pwm1;
		active_command_pwm2 = command.pwm2;
		motor_command_head = (motor_command_head + 1) % MOTOR_COMMAND_QUEUE_DEPTH;
		motor_command_count--;
	}
}

void data_class::UpdateMotorCommandOutput()
{
	// FNR neutral overrides the selected command source, but the actual motor
	// PWM still goes through the controlled DC-link-aware deceleration ramp.
	// The local FNR lever is only authoritative for on-board control; a
	// stale/neutral lever reading must not block active CAN commands.
	uint8_t entering_fnr_stop = 0;
	if(!sysFlag.canReady && fm2000_gpio.FNR1 == 3 && fabsf(active_command_pwm1) > 0.01f) {
		active_command_pwm1 = 0.0f;
		stop_ramp_active_mask |= 0x01;
		entering_fnr_stop = 1;
	}
	if(!sysFlag.canReady && fm2000_gpio.FNR2 == 3 && fabsf(active_command_pwm2) > 0.01f) {
		active_command_pwm2 = 0.0f;
		stop_ramp_active_mask |= 0x02;
		entering_fnr_stop = 1;
	}
	if(entering_fnr_stop && stop_vdc_reference <= 1.0f)
		stop_vdc_reference = pDcLink->ReadFastVoltage();

	inputRaw.source_pwm.f1 = active_command_pwm1;
	inputRaw.source_pwm.f2 = active_command_pwm2;
	// While the short brake owns a motor bridge the drive output is forced to
	// zero, but this ramp kept advancing, so release re-applied a large PWM
	// step to a stopped motor (measured 15 A inrush, 58 -> 37 V sag).
	// Restart the S-curve from zero instead.
	if(pPWM->IsShortBrakeActiveM1()) {
		currentPWM1 = 0.0f;
		accel_curve1.active = 0U;
	}
	if(pPWM->IsShortBrakeActiveM2()) {
		currentPWM2 = 0.0f;
		accel_curve2.active = 0U;
	}
	const uint8_t drive_requested =
			(fabsf(active_command_pwm1) > 0.01f || fabsf(active_command_pwm2) > 0.01f);
	pPWM->RequestDriveEnable(drive_requested, batt.measure_hall_current);
	if(drive_requested && pPWM->IsElectromagneticBrakeStarting()) {
		// Build direction-correct holding torque while applied, then retain it
		// during release settling. Normal acceleration starts afterwards.
		currentPWM1 = pPWM->LimitElectromagneticBrakeStartPwm(active_command_pwm1);
		currentPWM2 = pPWM->LimitElectromagneticBrakeStartPwm(active_command_pwm2);
		inputRaw.target_pwm.f1 = currentPWM1;
		inputRaw.target_pwm.f2 = currentPWM2;
		accel_curve1.active = accel_curve2.active = 0U;
		accel_curve1.elapsed_ms = accel_curve2.elapsed_ms = 0U;
		accel_last_update_ms1 = accel_last_update_ms2 = control_millis;
		return;
	}
	inputRaw.target_pwm.f1 = Calcu_motor_command_pwm(
			active_command_pwm1, fm2000_dir1, currentPWM1, fm2000_dir_change_wait1,
			accel_last_update_ms1, accel_curve1);
	inputRaw.target_pwm.f2 = Calcu_motor_command_pwm(
			active_command_pwm2, fm2000_dir2, currentPWM2, fm2000_dir_change_wait2,
			accel_last_update_ms2, accel_curve2);
}

void data_class::ClearMotorCommandQueue()
{
	motor_command_head = 0;
	motor_command_tail = 0;
	motor_command_count = 0;
	active_command_pwm1 = 0.0f;
	active_command_pwm2 = 0.0f;
	last_received_command_pwm1 = 0.0f;
	last_received_command_pwm2 = 0.0f;
	last_received_command_valid = 0;
	motor_command_10ms_count = 0;
	stop_ramp_active_mask = 0;
	stop_vdc_reference = 0.0f;
	accel_last_update_ms1 = control_millis;
	accel_last_update_ms2 = control_millis;
	accel_curve1 = {0.0f, 0.0f, 0U, 0U};
	accel_curve2 = {0.0f, 0.0f, 0U, 0U};
}

float data_class::GetStopVdcReference()
{
	return stop_vdc_reference;
}

float data_class::Calcu_limit(float throttle, float limit){
	float control_value = 0.0f;
	if (throttle >= 0.2f) {
	    if (limit >= 0.005f) {
	        control_value = 0.2f + (throttle - 0.2f) * (limit / 0.8f);// throttle 0.2~1.0 → control_value 0.2~1.0 (limit에 따라 스케일)
	    }
	    else  control_value =0;
	}
	else control_value=throttle;

	if(control_value>=1.0f) control_value=1.0f;
	return control_value;
}

//DoubleF_VALUE data_class::Calcu_sourcePWM(float i_poten, float limit, uint8_t jenhujin, uint8_t stop_lr){
//	DoubleF_VALUE o_poten;
//	float control_value=Calcu_limit(i_poten, limit);
//	o_poten.f1=o_poten.f2=control_value;
////	stabilize_change();
//	if(jenhujin){
//		switch(stop_lr){
//			case 3: break;
//			case 0:	o_poten.f1 =o_poten.f2 =0;break;//ALL press
//			case 1:	o_poten.f1 *=gamsok_ratio[gamsok_idx];break;
//			case 2:	o_poten.f2 *=gamsok_ratio[gamsok_idx];break;
//			default:break;
//		}
//	}
//	else//board_io.toggle.jenhujin==0 중립일때는 SPIN
//	{
//		switch(stop_lr)
//		{
//			case 3:	case 0:	o_poten.f1 =o_poten.f2 =0;break;//ALL press
//			case 1:	o_poten.f1 *=(-1.0f);o_poten.f2 *=(1.0f);break;//spin left?
//			case 2:	o_poten.f1 *=(1.0f); o_poten.f2 *=(-1.0f);break;//spin right
//			default:break;
//		}
//	}
//	//cprintf(C_RED, "i_poten[%.1f] limit[%.1f] control_value[%.1f] o_poten[%.1f|%.1f] jenhujin[%d] stop_lr[%d] gamsok_ratio[%.1f]\r\n",i_poten, limit, control_value, o_poten.f1, o_poten.f2,jenhujin, stop_lr, gamsok_ratio[gamsok_idx]);
//	return o_poten;
//}


//DoubleF_VALUE data_class::Calcu_targetPWM(float throttle, float limit,
//		uint8_t jenhujin_key, uint8_t jenhujin_confirmed,
//		uint8_t stop_lr, uint8_t m1_pol, uint8_t m2_pol,
//		float fwd_ratio, float bwd_ratio)
//{
//	// 1. source_pwm 계산 (스로틀, 좌우정지SW, 전후진KEY)
//	DoubleF_VALUE src = Calcu_sourcePWM(throttle, limit, jenhujin_key, stop_lr);
//
//	// 2. 전후진 비율 적용 (모터 극성 반영)
//	if(jenhujin_key == 1 || jenhujin_key == 2) {
//		uint8_t intent_fwd = (jenhujin_key == 1) ? 1 : 0;
//		uint8_t m1_fwd = intent_fwd ^ (m1_pol ? 1 : 0);
//		uint8_t m2_fwd = intent_fwd ^ (m2_pol ? 1 : 0);
//		src.f1 *= m1_fwd ? fwd_ratio : bwd_ratio;
//		src.f2 *= m2_fwd ? fwd_ratio : bwd_ratio;
//	}
//
//	// 3. FET 보호 / 방향전환 딜레이 시 PWM 강제 0
//	if(fet_temp_protect || JENHUJIN_SWITCH_CHANGE_FLAG) {
//		src.f1 = 0.0f;
//		src.f2 = 0.0f;
//	}
//	// CheckBrakeState용: 오버라이드 이후 저장 (방향전환 중에는 0으로 전달해야 stop 감지 가능)
//	inputRaw.source_pwm = src;
//
//	// 4. LogRamp 처리
//	currentPWM1 = PWM_UpdateRoutine(src.f1, currentPWM1, sysFlag.stop_throttle);
//	currentPWM2 = PWM_UpdateRoutine(src.f2, currentPWM2, sysFlag.stop_throttle);
//
//	// 5. 확정된 방향으로 부호 적용 (후진=음수)
//	DoubleF_VALUE result;
//	result.f1 = currentPWM1;
//	result.f2 = currentPWM2;
//	if(jenhujin_confirmed == 2) {
//		result.f1 *= -1.0f;
//		result.f2 *= -1.0f;
//	}
//	return result;
//}

void data_class::ON_Board_INPUT()
{
	//Get_AdcData();
	read_in_port();
	inputRaw.rpm.f1=pPWM->rpm.f1;
	inputRaw.rpm.f2=pPWM->rpm.f2;

	float requested_pwm1 = (fm2000_gpio.FNR1 == 1) ? adc_to_pwm(ladcValue[0]) :
	                       (fm2000_gpio.FNR1 == 2) ? -adc_to_pwm(ladcValue[0]) : 0.0f;
	float requested_pwm2 = (fm2000_gpio.FNR2 == 1) ? adc_to_pwm(ladcValue[1]) :
	                       (fm2000_gpio.FNR2 == 2) ? -adc_to_pwm(ladcValue[1]) : 0.0f;
	QueueMotorCommand(requested_pwm1, requested_pwm2);
	return;
}

void data_class::can_process_routine()
{
	// CAN motor commands (0x701 lift / 0x7C1 bangJaeCha) enter the same
	// command FIFO as on-board input; ramp/brake processing stays identical.
#if 0 // RX 확인용 로그 (필요 시 복원)
	static uint16_t rx_log_cnt = 0;
	if(rx_log_cnt < 5 || (rx_log_cnt % 20) == 0) {
		printf("#CAN RX type[%d] L[%d] R[%d] brkDly[%u] batt[%u] tgl[%02X] btn[%02X]\r\n",
				(int)systemControlType,
				vcu_sdu.LeftMotor_velocity, vcu_sdu.RightMotor_velocity,
				vcu_sdu.canBrakeDelay, vcu_sdu.canBattery,
				vcu_sdu.toggle.u8, vcu_sdu.btn.u8);
	}
	rx_log_cnt++;
#endif
	// Byte2는 전자브레이크 딜레이(raw x 5 ms, 기존 brkDly 스케일)로 사용하고
	// 가속 시간은 상수(ACCEL_DURATION_10MS_TICKS)로 고정한다.
	{
		uint16_t delay_ms = vcu_sdu.canBrakeDelay; // req[2] * 5
		if(delay_ms < EMB_DELAY_CAN_MIN_MS) delay_ms = EMB_DELAY_CAN_MIN_MS;
		if(delay_ms > EMB_DELAY_CAN_MAX_MS) delay_ms = EMB_DELAY_CAN_MAX_MS;
		brake_delay = delay_ms;
		can_accel_duration_10ms = ACCEL_DURATION_10MS_TICKS;
		can_decel_rpm_per_10ms = DECEL_RPM_PER_10MS;
	}
	inputRaw.io.btn = vcu_sdu.btn;
	inputRaw.io.toggle = vcu_sdu.toggle;
	if(vcu_sdu.btn.emergency) HOLD_Emergency = 1;
	QueueMotorCommand(vcu_sdu.LeftMotor_velocity / 1000.0f,
			vcu_sdu.RightMotor_velocity / 1000.0f);
}

LIFT_BUTTON data_class::Get_localGPIO(){
	LIFT_BUTTON btn;
	uint8_t rdata=0;
//		rdata|=!!(HAL_GPIO_ReadPin(pIO_AP_GPIO_Port, pIO_AP_Pin));
//		rdata<<=1;
//		rdata|=!!(HAL_GPIO_ReadPin(pIO_AN_GPIO_Port, pIO_AN_Pin));
//		rdata %=3;
//		btn.castLimit=rdata;

//		rdata=0;
//		rdata|=!!(HAL_GPIO_ReadPin(pIO_BP_GPIO_Port, pIO_BP_Pin));
//		rdata<<=1;
//		rdata|=!!(HAL_GPIO_ReadPin(pIO_BN_GPIO_Port, pIO_BN_Pin));
//		rdata %=3;
//		btn.castUPDN=rdata;
//
//		rdata=0;
//		rdata|=!!(HAL_GPIO_ReadPin(pIO_CP_GPIO_Port, pIO_CP_Pin));
//		rdata<<=1;
//		rdata|=!!(HAL_GPIO_ReadPin(pIO_CN_GPIO_Port, pIO_CN_Pin));
//		rdata %=3;
//		btn.liftUPDN=rdata;
	return btn;
}

void data_class::one_millisec_routine()
{
	update_short_brake_1ms();
}

void data_class::update_short_brake_1ms()
{
	if(pPWM == 0) return;

	// 제동 판단은 물리 FNR을 직접 보지 않고, 큐에서 실제 실행 중인
	// 공통 명령을 사용한다. 이후 CAN 등 다른 입력 경로도 동일하게 동작한다.
	uint8_t m1_neutral = (motor_command_class(active_command_pwm1) == 0);
	uint8_t m2_neutral = (motor_command_class(active_command_pwm2) == 0);

	float source_abs_m1 = fabsf(inputRaw.source_pwm.f1);
	float source_abs_m2 = fabsf(inputRaw.source_pwm.f2);
	float target_abs_m1 = fabsf(inputRaw.target_pwm.f1);
	float target_abs_m2 = fabsf(inputRaw.target_pwm.f2);
	// Let the normal drive PWM complete its 10 ms deceleration ramp first.
	// Enabling the short brake earlier makes Update_PWM() force drive PWM to
	// zero, so the calculated 500 ms ramp never reaches the motor bridge.
	const float ramp_complete_threshold = 0.01f;
	uint8_t ramp_complete_m1 = (target_abs_m1 <= ramp_complete_threshold);
	uint8_t ramp_complete_m2 = (target_abs_m2 <= ramp_complete_threshold);

	// After the drive ramp reaches zero, use the short brake only while the
	// neutral motor is still rotating. Release it once motion has stopped.
	uint8_t m1_moving_or_commanded =
			(fabsf(pPWM->rpm.f1) >= MOTOR_MIN_RPM)
			|| (source_abs_m1 > 0.05f) || (target_abs_m1 > 0.05f);
	uint8_t m2_moving_or_commanded =
			(fabsf(pPWM->rpm.f2) >= MOTOR_MIN_RPM)
			|| (source_abs_m2 > 0.05f) || (target_abs_m2 > 0.05f);
	uint8_t brake_m1 = m1_moving_or_commanded && m1_neutral && ramp_complete_m1;
	uint8_t brake_m2 = m2_moving_or_commanded && m2_neutral && ramp_complete_m2;
	uint8_t protection_active = over_current_protect || fet_temp_protect
			|| (over_current_retry_wait_10ms_cnt > 0) || !PWR_ON_Flg;

	if(protection_active) {
		pPWM->ResetShortBrakeControl();
		return;
	}

	float dc_link_fast = pDcLink->ReadFastVoltage();
	// The absolute DC-link hardware limit remains allowed to bypass the
	// normal ramp-first sequence.
	float hard_voltage = stop_vdc_reference * SHORT_BRAKE_VDC_HARD_RATIO;
	if(hard_voltage <= 1.0f || hard_voltage > SHORT_BRAKE_VDC_ABSOLUTE_HARD)
		hard_voltage = SHORT_BRAKE_VDC_ABSOLUTE_HARD;
	if(dc_link_fast >= hard_voltage) {
		brake_m1 = m1_moving_or_commanded;
		brake_m2 = m2_moving_or_commanded;
	}
	pPWM->UpdateShortBrakeControl_1ms(
			brake_m1, brake_m2, source_abs_m1, source_abs_m2,
			target_abs_m1, target_abs_m2,
			dc_link_fast);
}

void data_class::force_pwm_off_for_over_current()
{
	ClearMotorCommandQueue();
	inputRaw.target_pwm.f1 = 0.0f;
	inputRaw.target_pwm.f2 = 0.0f;
	inputRaw.source_pwm.f1 = 0.0f;
	inputRaw.source_pwm.f2 = 0.0f;
	currentPWM1 = 0.0f;
	currentPWM2 = 0.0f;
	if(pPWM != 0) {
		pPWM->Update_PWM(1, 0.0f, 0.0f);
		pPWM->brake_on_flag = 1;
	}
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);
}

void data_class::ten_millisec_routine()
{

	//printf("ten_millisec_routine can_TimeOut[%d]\r\n",pCAN->can_TimeOut);
	Get_AdcData();//always refresh diagnostics, including after a protection trip
#if SHUNT_OC_ENABLE
	if(pShuntOc != 0 && pShuntOc->IsTripped()) {
		if(PWR_ON_Flg) {
			printf("SHUNT TRIP: adc=%u thr=%u vdc=%.1f vthr=%.1f\r\n",
					pShuntOc->GetTripAdc(), pShuntOc->GetThresholdAdc(),
					pShuntOc->GetTripVoltage(), pShuntOc->GetVoltageThreshold());
		}
		PWR_ON_Flg = 0;
		gMAIN.relay.MC1 = 0;
		gMAIN.relay.MC2 = 0;
		force_pwm_off_for_over_current();
		error_code.tbd1 = 1;
		error_code.code = ERROR_CODE_7_SHUNT_OVER_CURRENT;
		if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_7_SHUNT_OVER_CURRENT);
		return;
	}
#endif
	//update_current();
	if(over_current_protect) {
		if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_9_OVER_CURRENT);
		return;
	}

#if WCS1600
	if(over_current_retry_wait_10ms_cnt > 0) {
		force_pwm_off_for_over_current();
		if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_9_OVER_CURRENT);
		over_current_retry_wait_10ms_cnt--;
		if(over_current_retry_wait_10ms_cnt == 0 && PWR_ON_Flg) {
			HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_SET);
			over_current_10ms_cnt = 0;
		}
		return;
	}

	if(batt.measure_hall_current >= MAX_CURRENT) {
		if(over_current_10ms_cnt < OVER_CURRENT_DETECT_10MS_TICK) over_current_10ms_cnt++;
		if(over_current_10ms_cnt >= OVER_CURRENT_DETECT_10MS_TICK) {
			over_current_retry_count++;
			error_code.over_current = 1;
			error_code.code = ERROR_CODE_9_OVER_CURRENT;
			if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_9_OVER_CURRENT);
			force_pwm_off_for_over_current();

			if(over_current_retry_count >= OVER_CURRENT_RETRY_MAX) {
				over_current_protect = 1;
				if(PWR_ON_Flg) power_off();
			}
			else {
				over_current_retry_wait_10ms_cnt = OVER_CURRENT_RETRY_WAIT_10MS_TICK;
			}
			return;
		}
	}
	else {
		over_current_10ms_cnt = 0;
	}
#endif

	if(fet_temp_protect) {
		if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_3_FET_OVER_TEMPERATURE);
		force_pwm_off_for_over_current();
		return;
	}
	local_lift=Get_localGPIO();//can과 상관없이 처리

	uint8_t motor_command_tick_50ms = 0;
	if(++motor_command_10ms_count >= 5) {
		motor_command_10ms_count = 0;
		motor_command_tick_50ms = 1;
	}

	 if(pCAN->can_TimeOut>0)pCAN->can_TimeOut--;
	 if(pCAN->can_TimeOut){
		sysFlag.canReady = 1;
	}
	else {
		if(sysFlag.canReady) {
			// A previously healthy drive link has been silent for the complete
			// timeout period.  Show code 8 immediately; no CAN frame is required
			// to keep this reason visible while the motor is safely stopped.
			pCAN->can_exist = 2;
			error_code.can_timeout = 1;
			error_code.code = ERROR_CODE_8_CAN_TIMEOUT;
			if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_8_CAN_TIMEOUT);
			printf("### ERROR 8: CAN DRIVE TIMEOUT, STM32 ALIVE t=%lu ms ESR=0x%08lX RF0R=0x%08lX ###\r\n",
					(unsigned long)control_millis,
					(unsigned long)hcan.Instance->ESR,
					(unsigned long)hcan.Instance->RF0R);
			// CAN 두절 시 마지막 CAN 버튼/토글(스프레이 등)이 래치되지 않도록 클리어.
			memset(&inputRaw.io, 0, sizeof(inputRaw.io));
			can_accel_rate = ACCEL_RATE;
			can_accel_duration_10ms = ACCEL_DURATION_10MS_TICKS;
			can_decel_rpm_per_10ms = DECEL_RPM_PER_10MS;
			brake_delay = ELECTROMAGNETIC_BRAKE_DELAY_MS;
		}
		sysFlag.canReady = 0;
		if(as_can_mode) {
			// AS 모드는 CAN 전용: 로컬 FNR 폴백 대신 정지 명령을 유지한다.
			if(motor_command_tick_50ms) QueueMotorCommand(0.0f, 0.0f);
		}
		// Do not enqueue 10 ms ADC intermediate values into a queue consumed
		// at 50 ms. That creates stale-command lag during a rapid stop.
		else if(motor_command_tick_50ms) ON_Board_INPUT();
	}
	// 명령 FIFO는 50 ms마다 하나씩 실행한다. 선택된 명령에 대한
	// PWM 램프/제동 출력 계산은 기존 10 ms 주기를 유지한다.
	if(motor_command_tick_50ms) ProcessMotorCommandQueue();
	UpdateMotorCommandOutput();

	if(current_state != prev_state) {
		state_change_flag_timeout = STABILIZE_DELAY_10MS_TICK;
	    prev_state = current_state; // 상태 변경 감지
	}

//++++++++++++
	float fet_temp_pwm_scale = get_fet_temp_pwm_scale();
	// PID feedback: keep the measured speed updated for every command source.
	// ON_Board_INPUT() only runs in local mode, so without this the CAN path
	// fed rpm=0 into the PID and it railed at +0.2 (over-speed).
	inputRaw.rpm.f1 = pPWM->rpm.f1;
	inputRaw.rpm.f2 = pPWM->rpm.f2;
	// Speed closed loop: PID on by default, master can disable per frame
	// via the 0x701 btn.disablePID bit (local mode: bit is 0 -> PID on).
	pPWM->Update_PWM(inputRaw.io.btn.disablePID,
			inputRaw.target_pwm.f1 * fet_temp_pwm_scale,
			inputRaw.target_pwm.f2 * fet_temp_pwm_scale);
	// Update_PWM writes the normal bridge state; restore controlled braking
	// immediately instead of leaving a full short until the next 1 ms tick.
	update_short_brake_1ms();
	// Electromagnetic brake sequence: after both PWM commands and measured
	// motor speeds have stopped, wait the configured 300 ms and apply it.
	// While a drive command is active, keep the stop detector released even
	// during the very small first portion of the S-curve.
	float brake_check_pwm1 = inputRaw.target_pwm.f1;
	float brake_check_pwm2 = inputRaw.target_pwm.f2;
	if(fabsf(active_command_pwm1) > 0.01f && fabsf(brake_check_pwm1) <= STOP_INPUT_PWM)
		brake_check_pwm1 = copysignf(STOP_INPUT_PWM + 0.01f, active_command_pwm1);
	if(fabsf(active_command_pwm2) > 0.01f && fabsf(brake_check_pwm2) <= STOP_INPUT_PWM)
		brake_check_pwm2 = copysignf(STOP_INPUT_PWM + 0.01f, active_command_pwm2);
	pPWM->CheckBrakeState(brake_check_pwm1, brake_check_pwm2);

//liftControl++
	if(inputRaw.io.btn.ioMsg==0)
	{
		LIFT_BUTTON llift;
		//printf("lift_control castUPDN[%d] lift_Xud[%d] castLimit[%d]\r\n",local_lift.castUPDN, local_lift.lift_Xud, local_lift.castLimit);
		//0 normal
		//1 downlimit
		//2 up limit
		//3 all limit
		//castUPDN=1:UP   2:DN
		llift.castUPDN=(local_lift.castUPDN)?local_lift.castUPDN:inputRaw.io.btn.castUPDN;
		if(local_lift.castLimit==2 && llift.castUPDN==1) llift.castUPDN=0;
		if(local_lift.castLimit==1 && llift.castUPDN==2) llift.castUPDN=0;
		if(local_lift.castLimit==3) llift.castUPDN=0;

		llift.liftUPDN=(local_lift.liftUPDN)?local_lift.liftUPDN:inputRaw.io.btn.lift_Xud;
		lift_control(llift.liftUPDN, llift.castUPDN);
	}
	//pSpary control
	if(inputRaw.io.btn.ioMsg==2 && PWR_ON_Flg){
		pSpary=inputRaw.io.toggle.spray1 &0x01;
		if(pSpary) HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_SET);
		else HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_RESET);
	}
	else if(!PWR_ON_Flg){
		pSpary=0;
		HAL_GPIO_WritePin(pRY4_GPIO_Port, pRY4_Pin, GPIO_PIN_RESET);
	}
//liftControl--

	// pRY3 powers the brake release coil. RESET=no power=DN/applied,
	// SET=powered=OFF/released. Protection/power-off always applies the brake.
	if(PWR_ON_Flg && !pPWM->brake_on_flag)
		HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);
}

void data_class::hnd_millisec_routine(){
	// UART3 is reserved for a machine-readable, fixed-column telemetry stream.
	// This routine is scheduled every 100 ms by HAL_SYSTICK_Callback().
	send_uart3_csv();
	if(sysFlag.SaveEEPROM){
		sysFlag.SaveEEPROM=0;
		printf("###SaveEEPROM ignored: SYSTEM_CONF disabled===\r\n");
	}
	if(pDataClass->sysFlag.canReady)CAN_DCU_Information();

	// 간단 상태(500ms): CAN으로 송신되는 RPM(0x5B8 스케일)과 CAN 수신 PWM 명령.
	static uint8_t simple_log_cnt = 0;
	if(++simple_log_cnt >= 5) {
		simple_log_cnt = 0;
		printf("FNR[%d,%d] txRPM[%d,%d] canPWM[%d,%d] tgt[%.2f,%.2f] pid[%.2f,%.2f] ofs[%.1f,%.1f] "
				"st[rdy%d pwr%d err%d ocp%d emg%d act%.2f,%.2f]\r\n",
				fm2000_gpio.FNR1, fm2000_gpio.FNR2,
				(int)(pPWM->rpm.f1 * 1.667f), (int)(pPWM->rpm.f2 * 1.667f),
				vcu_sdu.LeftMotor_velocity, vcu_sdu.RightMotor_velocity,
				inputRaw.target_pwm.f1, inputRaw.target_pwm.f2,
				pPWM->pid_pwm.f1, pPWM->pid_pwm.f2,
				pPWM->bemf_offset1, pPWM->bemf_offset2,
				(int)sysFlag.canReady, (int)PWR_ON_Flg, (int)error_code.code,
				(int)over_current_protect, (int)HOLD_Emergency,
				active_command_pwm1, active_command_pwm2);
	}

#if 0 // CAN 테스트 중 상태 로그 차단: #CAN RX 로그만 출력
	DoubleF_VALUE brake_duty = pPWM->GetShortBrakeDuty();
	float dc_link_fast_log = pDcLink->ReadFastVoltage();
	printf("FNR1[%d] FNR2[%d] error[%d] [%.1fV/%.2fA] Vfast[%.1f] Vref[%.1f] FT[%d] "
			"src[%.2f,%.2f] tgt[%.2f,%.2f] "
			"rpm[%.0f,%.0f] trpm[%.0f,%.0f] erpm[%.0f,%.0f] "
			"pid[%.3f,%.3f] drv[%.3f,%.3f] brk[%.3f,%.3f] "
			"hall[%.1f] Iraw[%u] adc[%u] m_current[%.1f,%.1f] "
			"ADC[%d,%d,%d,%d] OCraw[%u,%u] OC[%u/%u] OCv[%.1f/%.1f]\r\n",
			fm2000_gpio.FNR1,
			fm2000_gpio.FNR2,
			error_code.code,
			batt.measure_battery_voltage,
			batt.measure_hall_current,
			dc_link_fast_log,
			stop_vdc_reference,
			batt.fet_temp,
			inputRaw.source_pwm.f1,
			inputRaw.source_pwm.f2,
			inputRaw.target_pwm.f1,
			inputRaw.target_pwm.f2,
			pPWM->rpm.f1,pPWM->rpm.f2,
			pPWM->target_rpm.f1,pPWM->target_rpm.f2,
			pPWM->error_rpm.f1,pPWM->error_rpm.f2,
			pPWM->pid_pwm.f1,pPWM->pid_pwm.f2,
			pPWM->drive_pwm.f1,pPWM->drive_pwm.f2,
			brake_duty.f1,brake_duty.f2,
			batt.measure_hall_current,ladcValue[6],adc_buf[6],
			batt.measure_m1_current,batt.measure_m2_current,
			ladcValue[0],ladcValue[1],ladcValue[7],ladcValue[8],
			(pShuntOc != 0) ? pShuntOc->GetRawAdc() : 0,
			(pShuntOc != 0) ? pShuntOc->GetPan4RawAdc() : 0,
			(pShuntOc != 0) ? pShuntOc->GetTripAdc() : 0,
			(pShuntOc != 0) ? pShuntOc->GetThresholdAdc() : 0,
			(pShuntOc != 0) ? pShuntOc->GetTripVoltage() : 0.0f,
			(pShuntOc != 0) ? pShuntOc->GetVoltageThreshold() : 0.0f);
#endif
}

void data_class::send_uart3_csv()
{
	/*
	 * CSV protocol (115200 baud, 8-N-1):
	 * $VCU,time_ms,source,can_ready,adc1,adc2,fnr1,fnr2,toggle,button,
	 *      accel_raw,byte3_raw,can_cmd1,can_cmd2,target_pwm1,target_pwm2,
	 *      drive_pwm1,drive_pwm2,rpm_raw1,rpm_raw2,rpm_obs1,rpm_obs2,fet_temp,motor_temp,
	 *      voltage,current,emb_brake,error
	 * source: 0=GPIO/local, 1=CAN.  Prefixing each record with $VCU lets the
	 * receiver safely ignore boot messages or an incomplete first line.
	 */
	char line[280];
	const unsigned int source = sysFlag.canReady ? 1U : 0U;
	const unsigned int accel_raw = (unsigned int)can_byte2_raw;
	const unsigned int byte3_raw = (unsigned int)can_byte3_raw;
	int length = snprintf(line, sizeof(line),
			"$VCU,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.2f,%.2f,%u,%u\r\n",
			(unsigned long)control_millis, source, (unsigned int)sysFlag.canReady,
			(unsigned int)ladcValue[0], (unsigned int)ladcValue[1],
			(unsigned int)fm2000_gpio.FNR1, (unsigned int)fm2000_gpio.FNR2,
			(unsigned int)inputRaw.io.toggle.u8, (unsigned int)inputRaw.io.btn.u8,
			accel_raw, byte3_raw,
			(int)vcu_sdu.LeftMotor_velocity, (int)vcu_sdu.RightMotor_velocity,
			inputRaw.target_pwm.f1, inputRaw.target_pwm.f2,
			pPWM->drive_pwm.f1, pPWM->drive_pwm.f2,
			pPWM->rpm.f1, pPWM->rpm.f2,
			pPWM->observed_rpm.f1, pPWM->observed_rpm.f2,
			(int)batt.fet_temp, (int)batt.motor_temp,
			batt.measure_battery_voltage, batt.measure_hall_current,
			(unsigned int)(pPWM->brake_on_flag & 0x01U),
			(unsigned int)error_code.code);

	if(length <= 0) return;
	if(length >= (int)sizeof(line)) length = (int)sizeof(line) - 1;
	// A 220-byte line takes less than 20 ms at 115200 baud, safely below the
	// 100 ms reporting period.  A short timeout prevents monitoring from
	// delaying motor control if the peripheral ever becomes unavailable.
	HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)length, 25);
}

void data_class::onesec_routine()
{
	// UART1 heartbeat during a CAN-loss stop. If this continues, the STM32
	// main loop is alive and the fault is confined to the CAN link/sender.
	if(pCAN != 0 && pCAN->can_exist == 2) {
		printf("### ERROR 8 ACTIVE: CAN TIMEOUT, STM32 ALIVE t=%lu ms ESR=0x%08lX ###\r\n",
				(unsigned long)control_millis,
				(unsigned long)hcan.Instance->ESR);
	}
	batt.fet_temp=get_ntc_temperature(ladcValue[4]);
	batt.motor_temp=(int8_t)get_ntc_temperature(ladcValue[5]);
	// FET 과열 보호: limit_fet_temp 초과 1초 지속시 전원 차단
	if((int8_t)batt.fet_temp >= FET_TEMP_PWM_OFF) {
		if(!fet_temp_protect) {
			printf("### FET TEMP PWM OFF! [%d >= %d]\r\n", batt.fet_temp, FET_TEMP_PWM_OFF);
		}
		fet_temp_protect = 1;
		fet_temp_over_cnt = 0;
		error_code.fet_over_temperature = 1;
		error_code.code = ERROR_CODE_3_FET_OVER_TEMPERATURE;
		if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_3_FET_OVER_TEMPERATURE);
		force_pwm_off_for_over_current();
	}
	else if(fet_temp_protect && (int8_t)batt.fet_temp <= FET_TEMP_RESTART) {
		printf("### FET TEMP AUTO RESTART ENABLE! [%d <= %d]\r\n", batt.fet_temp, FET_TEMP_RESTART);
		fet_temp_protect = 0;
		fet_temp_over_cnt = 0;
		if(PWR_ON_Flg) HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_SET);
	}
	else if((int8_t)batt.fet_temp >= FET_TEMP_DERATE_START) {
		error_code.fet_over_temperature = 1;
		error_code.code = ERROR_CODE_3_FET_OVER_TEMPERATURE;
	}
	else {
		fet_temp_over_cnt = 0;
	}
//	if(pPWM->brake_on_flag)//No EMB_RelayOn
//	{
//		batt.measure_emb_resister=(float)get_emb_resister(ladcValue[7], ladcValue[6]);
//	}
//	batt.measure_battery_voltage=get_voltage(ladcValue[7]);
	update_error_code_once_per_second();
	display_error_code_once_per_second();

//	printf("FNR1[%d] FNR2[%d] error[%d] [%.1fV/%.2fA] FT[%d] src[%.2f,%.2f] tgt[%.2f,%.2f] [%lu,%lu] ADC[%d,%d,%d,%d]\r\n",
//			fm2000_gpio.FNR1,
//			fm2000_gpio.FNR2,
//			error_code.code,
//			batt.measure_battery_voltage,
//			batt.measure_m12_current,
//			batt.fet_temp,
//			inputRaw.source_pwm.f1,
//			inputRaw.source_pwm.f2,
//			inputRaw.target_pwm.f1,
//			inputRaw.target_pwm.f2,
//			ladcValue[2],ladcValue[3],
//			ladcValue[6],ladcValue[7],ladcValue[8],ladcValue[9]);


//printf("Potentio_val[%04d] limit[%04d] toggle[%02d] mi_dir[%d] m2_dir[%d]\r\n", Potentio_val, vcu_sdu.limit, vcu_sdu.toggle.u8, sysFlag.motor_dir1, sysFlag.motor_dir2);
//	printf("brake_delay[%04d]  use_battery_voltage[%.1f] measure_emb_resister[%.1f] currentA[%.1f+%.1f=%.1f]\r\n",
//			brake_delay, batt.use_battery_voltage,batt.measure_emb_resister, batt.measure_m1_current, batt.measure_m2_current, batt.measure_m12_current);
//	printf("ex_pwm1[%04d] ex_pwm1[%04d] state[%d]\r\n",gMAIN.ex_pwm1,  gMAIN.ex_pwm2, gMAIN.flg_state.u8);
//	printf("HOLD_Emergency[%d] emb_delay[%d] forward[%.1f^%.1f]\r\n", HOLD_Emergency,brake_delay, set_foreward, set_backward);
//	printf("lift_Xud[%d] lift_Wud[%d]\r\n", board_io.btn.lift_Xud, board_io.btn.castUPDN);
//	printf("stop[%d][%d]\r\n", gMAIN.flg_state.stop1, gMAIN.flg_state.stop2);
//	printf("fet_temp[%d/%d] motor_temp[%d/%d] battery_voltage[%.f|%d]\r\n",batt.fet_temp,ladcValue[4], batt.motor_temp,ladcValue[5], batt.measure_battery_voltage, ladcValue[7]);
#if 0
	printf("[CONF] batt_v=%d lim_I=%d lim_mT=%d lim_fT=%d alrm_B=%d cart=%d pol1=%d pol2=%d\r\n",
		sysConf.battery_voltage, sysConf.limit_current, sysConf.limit_motor_temp, sysConf.limit_fet_temp,
		sysConf.alarm_Battery, sysConf.cart_type, sysConf.motor1_polarity, sysConf.motor2_polarity);
	printf("[CONF] tbd=%d offset=%d slip=%d fwd=%d bwd=%d accel=%d decel=%d brk_dly=%d brk_rate=%d\r\n",
		sysConf.tbd, sysConf.tottle_offset, sysConf.stop_slip,
		sysConf.foreward, sysConf.backward, sysConf.accel, sysConf.decel,
		sysConf.brake_delay, sysConf.brake_rate);
//	printf("vr[%.2f/%.2f] rpm[%.1f|%.1f] target_rpm[%.1f|%.1f] eRPM[%.1f|%.1f] \r\n",
//			inputRaw.source_pwm.f1, inputRaw.source_pwm.f2, inputRaw.rpm.f1, inputRaw.rpm.f2,pPWM->target_rpm.f1, pPWM->target_rpm.f2, pPWM->error_rpm.f1, pPWM->error_rpm.f2);
#endif
	//relay_control();
}
//
//void data_class::packRPM(int16_t rpm1, int16_t rpm2, uint8_t current, uint8_t out[4])
//{
//    // 1️ 범위 클램프
//    if (rpm1 > 3600) rpm1 = 3600;
//    if (rpm1 < -3600) rpm1 = -3600;
//    if (rpm2 > 3600) rpm2 = 3600;
//    if (rpm2 < -3600) rpm2 = -3600;
//
//    // 2️ 12bit로 스케일 변환 (0~4095)
//    uint16_t enc1 = (uint16_t)((rpm1 + 3600) * 4095L / 7200L);
//    uint16_t enc2 = (uint16_t)((rpm2 + 3600) * 4095L / 7200L);
//
//    // 3️ 비트 패킹
//    out[0] = (enc1 >> 4) & 0xFF;
//    out[1] = ((enc1 & 0x0F) << 4) | ((enc2 >> 8) & 0x0F);
//    out[2] = enc2 & 0xFF;
//    out[3] = current;
//}
//
//void data_class::unpackRPM(uint8_t in[4], int16_t &rpm1, int16_t &rpm2, uint8_t &current)
//{
//    uint16_t enc1 = ((uint16_t)in[0] << 4) | ((in[1] >> 4) & 0x0F);
//    uint16_t enc2 = ((uint16_t)(in[1] & 0x0F) << 8) | in[2];
//
//    current = in[3];
//
//    // 스케일 복원
//    rpm1 = (int16_t)((enc1 * 7200L / 4095L) - 3600);
//    rpm2 = (int16_t)((enc2 * 7200L / 4095L) - 3600);
//}


// ===============================
// 7-byte pack/unpack
// [0..3] rpm1(12) rpm2(12) current(8)
// [4..6] battery(10) motor_temp(7) fet_temp(7)
// ===============================
static inline int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t hi) {
  return (v > hi) ? hi : v;
}

static inline uint8_t clamp_u8(uint8_t v, uint8_t hi) {
  return (v > hi) ? hi : v;
}

// rpm: -3600..+3600  <->  enc: 0..4095
static inline uint16_t rpm_to_12bit(int16_t rpm) {
  rpm = clamp_i16(rpm, -3600, 3600);
  // enc = (rpm + 3600) * 4095 / 7200
  return (uint16_t)(((int32_t)(rpm + 3600) * 4095L) / 7200L);
}

static inline int16_t bit12_to_rpm(uint16_t enc) {
  enc &= 0x0FFF;
  // rpm = enc * 7200 / 4095 - 3600
  return (int16_t)(((int32_t)enc * 7200L) / 4095L - 3600L);
}

void data_class::pack7(uint8_t out[7], int16_t rpm1, int16_t rpm2, uint8_t current, uint16_t batt10, uint8_t motor_temp7, uint8_t fet_temp7)
{
  // ---- clamp ----
  uint16_t enc1 = rpm_to_12bit(rpm1);
  uint16_t enc2 = rpm_to_12bit(rpm2);

  current      = clamp_u8(current, 255);
  batt10       = clamp_u16(batt10, 1023);      // 10-bit
  motor_temp7  = clamp_u8(motor_temp7, 127);   // 7-bit
  fet_temp7    = clamp_u8(fet_temp7, 127);     // 7-bit

  // ---- [0..3] rpm/current ----
  out[0] = (enc1 >> 4) & 0xFF;
  out[1] = (uint8_t)(((enc1 & 0x0F) << 4) | ((enc2 >> 8) & 0x0F));
  out[2] = (uint8_t)(enc2 & 0xFF);
  out[3] = current;

  // ---- [4..6] batt/temp ----
  // b4: batt[9:2]
  out[4] = (uint8_t)((batt10 >> 2) & 0xFF);

  // b5: batt[1:0] (bits7..6) | motor_temp[6:1] (bits5..0)
  out[5] = (uint8_t)(((batt10 & 0x03) << 6) | ((motor_temp7 >> 1) & 0x3F));

  // b6: motor_temp[0] (bit7) | fet_temp[6:0] (bits6..0)
  out[6] = (uint8_t)(((motor_temp7 & 0x01) << 7) | (fet_temp7 & 0x7F));
}

void data_class::unpack7(const uint8_t in[7], int16_t &rpm1, int16_t &rpm2, uint8_t &current, uint16_t &batt10, uint8_t &motor_temp7, uint8_t &fet_temp7)
{
  // ---- [0..3] rpm/current ----
  uint16_t enc1 = ((uint16_t)in[0] << 4) | ((in[1] >> 4) & 0x0F);
  uint16_t enc2 = ((uint16_t)(in[1] & 0x0F) << 8) | (uint16_t)in[2];

  current = in[3];

  rpm1 = bit12_to_rpm(enc1);
  rpm2 = bit12_to_rpm(enc2);

  // ---- [4..6] batt/temp ----
  batt10 = ((uint16_t)in[4] << 2) | ((in[5] >> 6) & 0x03);

  motor_temp7 = (uint8_t)(((in[5] & 0x3F) << 1) | ((in[6] >> 7) & 0x01));
  motor_temp7 &= 0x7F;

  fet_temp7 = (uint8_t)(in[6] & 0x7F);
}


void data_class::CAN_DCU_Information(){
	CAN_UP_FLAG_DATA upFlag;
	// 0x5B8 = CAN_From_VCU: 아성 ESP32 인터페이스와 참조 프로젝트 공통 상태 ID.
	uint32_t Pub_ID=0x5B8;
	uint8_t buf[8]={0,};
	uint8_t pack_buf[7]={0,};

	// 전송용 RPM 보정: 전진/후진 동일 배율
	// 제어 변수 pPWM->rpm 은 변경하지 않음
	int16_t rpm1 = (int16_t)(pPWM->rpm.f1 * 1.667f);
	int16_t rpm2 = (int16_t)(pPWM->rpm.f2 * 1.667f);
	uint8_t current=(uint16_t)batt.measure_hall_current;//unit A
	uint16_t batt10=(uint16_t)(batt.measure_battery_voltage*10)+5;//0.5V보정
	uint8_t motor_temp7=(uint8_t)batt.motor_temp;
	uint8_t fet_temp7=(uint8_t)batt.fet_temp;

	pack7(pack_buf, rpm1, rpm2, current, batt10, motor_temp7, fet_temp7);
	unpack7(pack_buf, rpm1, rpm2, current, batt10, motor_temp7, fet_temp7);
	//packRPM((int16_t)inputRaw.rpm.f1, (int16_t)ctl_pwm.rpm.f2, current,  rpm_buf);
	//printf("packRPM rpm_buf[%x][%x][%x][%x]\r\n", rpm_buf[0], rpm_buf[1], rpm_buf[2], rpm_buf[3]);
	//printf("unpackRPM rpm1[%d] rpm2[%d] current[%d] batt10[%d]\r\n", rpm1, rpm2, current, batt10);
	//printf("CAN_DCU_Information[%lx]\r\n", Pub_ID);//pDataClass->vcu_sdu.btn.u8

	upFlag.bisang=HOLD_Emergency;//sysFlag.bisang;
	upFlag.emb=pPWM->brake_on_flag&0x01;
//batt.measure_emb_resister
//no connect emb-> 6.0~6.5
//1 connect emb-> 1.49
//2 connect emb-> 1.35
	upFlag.emb_error=0;// (batt.measure_emb_resister>4.0)? 1:0;
	upFlag.can_ready=1;

	if(inputRaw.io.btn.ioMsg==2){
		upFlag.limitSwitch=local_lift.liftUPDN;//방제차-limitSwitch
		upFlag.spray=pSpary;
	}
//(4byte-32bit) 12bit+12bit+8bit  rpm1(12bit) + rpm2(12bit) + current(8bit)
//(3byte-24bit) measure_battery_voltage(10bit)+motor_temp(7bit)+fet_temp(7bit)
	buf[0]=pack_buf[0];
	buf[1]=pack_buf[1];
	buf[2]=pack_buf[2];
	buf[3]=pack_buf[3];//HI_UINT16((uint16_t)batt.measure_battery_voltage);
	buf[4]=pack_buf[4];//LO_UINT16((uint16_t)batt.measure_battery_voltage);
	buf[5]=pack_buf[5];//(uint8_t)batt.motor_temp;
	buf[6]=pack_buf[6];//(uint8_t)batt.fet_temp;
	buf[7]=upFlag.u8;//sysFlag.u8;
	pCAN->put_canTxd(Pub_ID, buf);
}

void data_class::MakeDCU_Data()
{
	uint8_t pData[32]={0,};
	DCU_DATA_STRUCTURE ldcu;
	uint8_t i,sum;
	ldcu.stx=0xAF;
	//ldcu.id=ZIGBEE_MY_ID;
	ldcu.leftMotor_velocity = Potentio_val;//gMAIN.adcValue[0];
	ldcu.limit_velocty = limit_val;//gMAIN.adcValue[1];
	ldcu.Vbat=(int16_t)batt.measure_battery_voltage;//gMAIN.battery_voltage;
	ldcu.ntc_fet=(int8_t)batt.fet_temp;
	ldcu.ntc_mot=(int8_t)batt.motor_temp;
	ldcu.rssi=0xDA;
	ldcu.dbtn1.emergency=HOLD_Emergency&0x01;
	ldcu.dbtn2.u8=0;
	ldcu.dummy1=0xAA;
	ldcu.checksum=0x00;
	ldcu.etx=0x40;
//checksum++
	//memcpy(pData, &dcu_data, 16);
	memcpy(pData, &ldcu, 16);
	sum=0;
	for(i=0;i<13;i++)sum=sum+pData[i];
	sum=sum&0xFF;
	ldcu.checksum=sum;
//checksum--
	memcpy(&dcu_data, &ldcu, 16);

#if 0
	memcpy(pData, &dcu_data, 16);
	printf("sum[%x] checksume[%x] [%x] [%02X][%02X][%02X][%02X]-[%02X][%02X][%02X][%02X]-[%02X][%02X][%02X][%02X]-[%02X][%02X][%02X][%02X]\r\n",
			sum, pData[14],dcu_data.checksum,  pData[0],pData[1],pData[2],pData[3],pData[4],pData[5],pData[6],pData[7],pData[8],pData[9],pData[10],pData[11],pData[12],pData[13],pData[14],pData[15]);
	//printf("speed[%d] limit[%d]\r\n",lrcu.speed, lrcu.limit);
#endif

}
void data_class::Get_AdcData()
{
	uint16_t adc[16]={0,};
	for(int i=0;i<10;i++){
		adc[i]=adc_buf[i];
	}
	memcpy(ladcValue,adc,32);
	ladcValue[0]=get_m0_filter(adc[0]);
	ladcValue[1]=get_m1_filter(adc[1]);
	ladcValue[2]=get_m2_filter(adc[2],0.05f);//over current detect
	ladcValue[3]=get_m3_filter(adc[3],0.05f);//over current detect
	ladcValue[6]=get_m6_filter(adc[6],0.1f);//wcs current detect
	ladcValue[7]=get_m7_filter(adc[7],0.1f);//dc-link voltage detect
	ladcValue[8]=get_m8_filter(adc[8],0.05f);//bemf detect
	ladcValue[9]=get_m9_filter(adc[9],0.05f);//bemf detect
	batt.measure_battery_voltage=dc_link::AdcToVoltage(ladcValue[7]);
#if WCS1600
	batt.measure_hall_current = fabsf(get_wcs1600_current(ladcValue[6]));
#else
	batt.measure_hall_current = 0.0f;
#endif
	batt.measure_m1_current=(ladcValue[2]>ladcValue[3]) ? get_voltage(ladcValue[2]):get_voltage(ladcValue[3]);
	batt.measure_m2_current=batt.measure_m1_current;
	//batt.measure_m1_current=get_voltage(ladcValue[2]);
	//batt.measure_m2_current=get_voltage(ladcValue[3]);

}

float data_class::get_voltage(uint16_t value)
{
	float voltage = (float)value * 3.3f / 0xfff; // 읽은 센서값을 전압으로 변경
	voltage=voltage*520;//51K 1K x10
    return voltage/10.0f;
}

//float data_class::get_emb_resister(uint16_t acc_adc, uint16_t value)
//{
//	uint16_t Remb;
//	uint16_t Vacc=get_voltage(acc_adc);
//	uint16_t Vemb=get_voltage(value);
//
//	//float v = (float)value * 3.3f / 0xfff; // 읽은 센서값을 전압으로 변경
//
//    //v=v*520;//51K 1K x10
//    //Remb=(uint16_t)v;//voltage
//
//    return Remb;
//}


float data_class::get_emb_resister(uint16_t acc_adc, uint16_t emb_adc)
{
    float Vacc=	(float)acc_adc * 3.3f / 0xfff; // 읽은 센서값을 전압으로 변경
    Vacc=Vacc*52;//51K 1K x10
    float Vemb=	(float)emb_adc * 3.3f / 0xfff; // 읽은 센서값을 전압으로 변경
    Vemb=Vemb*52;//51K 1K x10
    float RatioEMB=Vemb/Vacc;
	//printf("@@@acc_adc[%d] emb_adc[%d] Vacc[%.2f] Vemb[%.2f] RatioEMB[%.2f]\r\n", acc_adc, emb_adc, Vacc, Vemb, RatioEMB);
    return RatioEMB;
}

int16_t data_class::FormSensorMnt_CalCPUTemp(uint16_t value)
{
    float t;
    static int16_t bf[10] = {0};
    static uint8_t i = 0;
    uint8_t j;
    int16_t sum = 0;

    t = (float)value * 3.3f / 0xfff; // 읽은 센서값을 전압으로 변경
    // printf("1)value[%d] voltage[%.2f]\r\n", value , t);
    t = (1.43 - t) / 0.0043 + 25.0; // 슬로프오 오프셋을 계산
    //printf("2)value[%d] voltage[%.2f]\r\n", value , t);
    bf[i] = (int16_t)t; // 이동평균 계산을 위한 버퍼에 저장
    i++;
    i %= 10;
    sum = 0;
    for (j = 0; j < 10; j++) // 이동 평균 계산.
    {
        sum += bf[j];
    }
    sum /= 10;
    return sum;
}

uint16_t data_class::get_m0_filter(uint16_t adc)
{
  static float m0_value;
  m0_value=(m0_value*(1-ADC_SENSITIVITY))+(adc*ADC_SENSITIVITY);
  return (uint16_t)m0_value;
}

uint16_t data_class::get_m1_filter(uint16_t adc)
{
  static float m1_value;
  m1_value=(m1_value*(1-ADC_SENSITIVITY))+(adc*ADC_SENSITIVITY);
  return (uint16_t)m1_value;
}

uint16_t data_class::get_m2_filter(uint16_t adc, float senstivity)
{

  static float m2_value;
  m2_value=(m2_value*(1-senstivity))+(adc*senstivity);
  return (uint16_t)m2_value;
}

uint16_t data_class::get_m3_filter(uint16_t adc, float senstivity)
{
  static float m3_value;
  m3_value=(m3_value*(1-senstivity))+(adc*senstivity);
  return (uint16_t)m3_value;
}

uint16_t data_class::get_m6_filter(uint16_t adc, float senstivity)
{
  static float m6_value;
  m6_value=(m6_value*(1-senstivity))+(adc*senstivity);
  return (uint16_t)m6_value;
}

uint16_t data_class::get_m7_filter(uint16_t adc, float senstivity)
{
  static float m7_value;
  m7_value=(m7_value*(1-senstivity))+(adc*senstivity);
  return (uint16_t)m7_value;
}

uint16_t data_class::get_m8_filter(uint16_t adc, float senstivity)
{
  static float m8_value;
  m8_value=(m8_value*(1-senstivity))+(adc*senstivity);
  return (uint16_t)m8_value;
}
uint16_t data_class::get_m9_filter(uint16_t adc, float senstivity)
{
  static float m9_value;
  m9_value=(m9_value*(1-senstivity))+(adc*senstivity);
  return (uint16_t)m9_value;
}

//float data_class::get_wcs1600_current_ma(uint16_t adc)
//{
//	// WCS1600 board calibration: 1925 -> 130mA, 1940 -> 880mA.
//	const float adc_zero = 1922.4f;
//	const float ma_per_adc = 50.0f;
//	float current_ma = ((float)adc - adc_zero) * ma_per_adc;
//	if(current_ma < 0.0f) current_ma = 0.0f;
//	return current_ma;
//}

float data_class::get_wcs1600_current(uint16_t adc)
{
    const float VREF = 3.3f;
    const float ADC_MAX = 4095.0f;
    // WCS1600 current calibration:
    // displayed 12 A = reference ammeter 4 A, therefore the sensitivity
    // must be increased by 12/4 to reduce the calculated current to 1/3.
    // 6.11 mV/A * 3 = 18.33 mV/A.
    const float SENSITIVITY = 0.01833f;
    float zero_voltage = (wcs1600_zero_adc * VREF) / ADC_MAX;
    float voltage = ((float)adc * VREF) / ADC_MAX;

    return (voltage - zero_voltage) / SENSITIVITY;
}

void data_class::calibrate_wcs1600_zero()
{
#if WCS1600
	// power_on() calls this while SD is LOW, so bridge current is zero.
	// Average the live DMA channel instead of the slower filtered value.
	uint32_t sum = 0;
	const uint16_t samples = 100;
	for(uint16_t i = 0; i < samples; i++) {
		sum += adc_buf[6];
		HAL_Delay(1);
	}
	const float measured_zero = (float)sum / (float)samples;
	// A valid zero output must remain inside the ADC range and reasonably
	// close to mid-scale. Keep the calibrated fallback if wiring is invalid.
	if(measured_zero >= 1200.0f && measured_zero <= 2800.0f)
		wcs1600_zero_adc = measured_zero;
	printf("WCS1600 zero calibrated: %.1f ADC\r\n", wcs1600_zero_adc);
#endif
}

void data_class::update_current()
{
	batt.measure_hall_current = fabsf(get_wcs1600_current(ladcValue[6]));
	//batt.measure_m1_current = batt.measure_hall_current;
	//batt.measure_m2_current = batt.measure_hall_current;
}

void data_class::update_error_code_once_per_second()
{
	float measured_voltage = batt.measure_battery_voltage;
	float low_voltage = (pDcLink != 0)
			? pDcLink->GetLowVoltageLimit() : BATTERY_24V_MIN_VOLTAGE;
	float over_voltage = (pDcLink != 0)
			? pDcLink->GetHighVoltageLimit() : BATTERY_24V_MAX_VOLTAGE;

	memset(&error_code,0,sizeof(ERROR_CODE_STATE));
	error_code.low_voltage = (measured_voltage < low_voltage);
	error_code.over_voltage = (measured_voltage > over_voltage);
	error_code.fet_over_temperature = (fet_temp_protect || (int8_t)batt.fet_temp >= FET_TEMP_DERATE_START);
	error_code.motor_over_temperature = ((int8_t)batt.motor_temp > LIMIT_MOTOR_TEMP);
	error_code.motor1_fault = batt.moter_error.m1_error;
	error_code.motor2_fault = batt.moter_error.m2_error;
#if SHUNT_OC_ENABLE
	error_code.tbd1 = (pShuntOc != 0) ? pShuntOc->IsTripped() : 0;
#else
	error_code.tbd1 = 0;
#endif
	error_code.can_timeout = (pCAN != 0 && pCAN->can_exist == 2);
#if WCS1600
	error_code.over_current = (over_current_protect || batt.measure_hall_current >= MAX_CURRENT);
#else
	error_code.over_current = over_current_protect;
#endif

	if(error_code.over_current) error_code.code = ERROR_CODE_9_OVER_CURRENT;
	else if(error_code.fet_over_temperature) error_code.code = ERROR_CODE_3_FET_OVER_TEMPERATURE;
	else if(error_code.tbd1) error_code.code = ERROR_CODE_7_SHUNT_OVER_CURRENT;
	else if(error_code.low_voltage) error_code.code = ERROR_CODE_1_LOW_VOLTAGE;
	else if(error_code.over_voltage) error_code.code = ERROR_CODE_2_OVER_VOLTAGE;
	else if(error_code.motor_over_temperature) error_code.code = ERROR_CODE_4_MOTOR_OVER_TEMPERATURE;
	else if(error_code.motor1_fault) error_code.code = ERROR_CODE_5_MOTOR1_FAULT;
	else if(error_code.motor2_fault) error_code.code = ERROR_CODE_6_MOTOR2_FAULT;
	else if(error_code.can_timeout) error_code.code = ERROR_CODE_8_CAN_TIMEOUT;
	else {
		error_code.code = ERROR_CODE_0_NORMAL;
		error_code.normal = 1;
	}
}

void data_class::display_error_code_once_per_second()
{
	if(pFND595 == 0) return;

	if(error_code.code != ERROR_CODE_0_NORMAL) {
		fnd_can_dot_state = 0;
		pFND595->PrintDigit(error_code.code);
		return;
	}

	// Normal display is the vehicle mode selected at boot. The decimal point
	// is now exclusively a CAN-alive indicator; it no longer indicates the
	// internally detected 24/48 V protection range.
	if(sysFlag.canReady) fnd_can_dot_state ^= 1U;
	else fnd_can_dot_state = 0;
	// PA8 fixes AS mode when fitted. On legacy AS boards without that strap,
	// an accepted 0x701/0x7C1 frame also selects the A display. Only an
	// accepted 0x702 frame selects F.
	const uint8_t display_as = as_can_mode
			|| systemControlType == AS_CONTROL
			|| systemControlType == SAMBOO_LIFT
			|| systemControlType == SAMBOO_BANGJAE;
	pFND595->PrintHex(display_as ? 0x0AU : 0x0FU, fnd_can_dot_state);
}

int8_t data_class::get_ntc_temperature(uint16_t adc)
{
#define ADC_FULL_SCALE 330 //3.3V
#define PARAMETER_BETA 3435.0f
#define RES25oC        10000.0f  //25도 써미스트값
#define R10_RES        10000.0f  //ohm 10k 1%
  float ohm;
  float fV,tempC;
  int8_t temperaure;

  fV=adc*(ADC_FULL_SCALE/4096.0f);//12bit
 // ohm=((ADC_FULL_SCALE/fV)-1)*R10_RES;//ohm
  ohm=(fV*R10_RES)/(ADC_FULL_SCALE-fV);//ohm
  tempC =(PARAMETER_BETA/(log(ohm/RES25oC)+(PARAMETER_BETA/(273.15f+25.0f)))) -273.15f;
  if(tempC<0)tempC=0;
  if(tempC>250)tempC=250;
  temperaure=(int8_t)tempC;
#if 0
 printf("adc[%d] fVx100[%.2f] ohm[%.1f] temp[%.2f] temperaure[%d]\r\n", adc, fV, ohm, tempC,temperaure);
#endif
  return temperaure;
}

uint8_t data_class::get_switch_port(uint8_t port){
	uint8_t rdata=0;
	return rdata;
}

void data_class::get_fm2000_port(){
	uint8_t rdata=0;
	rdata|=!!(HAL_GPIO_ReadPin(pIO_AP_GPIO_Port, pIO_AP_Pin));
	rdata<<=1;
	rdata|=!!(HAL_GPIO_ReadPin(pIO_AN_GPIO_Port, pIO_AN_Pin));
	rdata %=4;
	fm2000_gpio.FNR1=rdata;

	rdata=0;
	rdata|=!!(HAL_GPIO_ReadPin(pIO_BP_GPIO_Port, pIO_BP_Pin));
	rdata<<=1;
	rdata|=!!(HAL_GPIO_ReadPin(pIO_BN_GPIO_Port, pIO_BN_Pin));
	rdata %=4;
	fm2000_gpio.FNR2=rdata;
	fm2000_gpio.emergency=0;//!!(HAL_GPIO_ReadPin(pIO_DN_GPIO_Port, pIO_DN_Pin));
}

void data_class::read_in_port(){

	uint8_t rdata=0;
	get_fm2000_port();
	if(fm2000_gpio.emergency==1) HOLD_Emergency=1;
}

int data_class::clamp(int value, int min, int max) {
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}
