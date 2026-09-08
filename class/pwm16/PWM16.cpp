/*
 * PWM16.cpp
 *
 *  Created on: Dec 31, 2023
 *      Author: thpark
 */
#include "extern.h"
#include "PWM16.h"
#include <math.h>


PWM16 *pPWM;

#define INTEGRAL_MAX_LIMIT 500.0f   // 적분값 최대 제한 (Ki 기여 최대 0.024 수준 유지)
static float ShortBrakeReference(float fallback_voltage)
{
	float reference = (pDataClass != 0) ? pDataClass->GetStopVdcReference() : 0.0f;
	return (reference > 1.0f) ? reference : fallback_voltage;
}

static float ShortBrakeThreshold(float reference, float ratio)
{
	float threshold = reference * ratio;
	if(threshold > SHORT_BRAKE_VDC_ABSOLUTE_HARD)
		threshold = SHORT_BRAKE_VDC_ABSOLUTE_HARD;
	return threshold;
}

float PWM16::PID_Compute1(float error) {
	float kp=pidCONF.Kp;
	//float derivative1 = error - prev_error1;
	//float rpm_to_pwm_scale = 1.0f / MOTOR_MAX_RPM;  // 3600rpm 기준이면 약 0.00028

	// 적분 Anti-windup: 적분값이 너무 커지지 않도록 제한
	integral1 += error;
	
	// 적분값 제한 (Anti-windup)
	if (integral1 > INTEGRAL_MAX_LIMIT) integral1 = INTEGRAL_MAX_LIMIT;
	if (integral1 < -INTEGRAL_MAX_LIMIT) integral1 = -INTEGRAL_MAX_LIMIT;
	
    prev_error1 = error;
    //float error = target_rpm - current_rpm;
    //float pwm = rpm_to_pwm_scale * kp * error;
    float pwm = pidCONF.rpm_to_pwm_scale * (kp * error+ pidCONF.Ki * integral1);
    //float pwm = rpm_to_pwm_scale * (kp * error+ ki * integral + kd * derivative);
    return pwm;
    //return kp * error + ki * integral + kd * derivative;
}

float PWM16::PID_Compute2(float error) {

	float kp=pidCONF.Kp;
    //float derivative2 = error - prev_error2;
    //pDataClass->rpm_to_pwm_scale = 1.0f / MOTOR_MAX_RPM;  // 3600rpm 기준이면 약 0.00028
    
    integral2 += error;
    
    // 적분값 제한 (Anti-windup)
	if (integral2 > INTEGRAL_MAX_LIMIT) integral2 = INTEGRAL_MAX_LIMIT;
	if (integral2 < -INTEGRAL_MAX_LIMIT) integral2 = -INTEGRAL_MAX_LIMIT;
    
    prev_error2 = error;
    //float error = target_rpm - current_rpm;
    //float pwm = rpm_to_pwm_scale * kp * error;
    float pwm = pidCONF.rpm_to_pwm_scale * (kp * error+ pidCONF.Ki * integral2);
    //float pwm = rpm_to_pwm_scale * (kp * error+ ki * integral + kd * derivative);
    return pwm;
    //return kp * error + ki * integral + kd * derivative;
}

PWM16::PWM16()
{
	// TODO Auto-generated constructor stub
	PWM16_START();
	pidCONF.Ki=0.1f;
	pidCONF.Kd=0.1f;
	integral1=integral2=0;
	m1_value=0.0f;
	m2_value=0.0f;  // 초기화 추가
	bemf_offset1=0.0f;
	bemf_offset2=0.0f;
	rpm.f1=0.0f;
	rpm.f2=0.0f;  // 재부팅 직후 쓰레기값 방지
	observed_rpm.f1=0.0f;
	observed_rpm.f2=0.0f;
	drive_pwm.f1=0.0f;
	drive_pwm.f2=0.0f;
}

PWM16::~PWM16()
{
	// TODO Auto-generated destructor stub
}

void PWM16::PWM16_START()
{

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	//PWM_Enable(1);
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_SET);
}


void PWM16::DisableAllFETs_Dual(){
	htim2.Instance->CCR1=0;
	htim2.Instance->CCR2=1024;//invert
	htim2.Instance->CCR3=0;
	htim2.Instance->CCR4=1024;//invert

	htim3.Instance->CCR1=0;
	htim3.Instance->CCR2=1024;//invert
	htim3.Instance->CCR3=0;
	htim3.Instance->CCR4=1024;//invert
}

void PWM16::Calibrate_BEMF_Offset() {
    float adc_raw1, adc_raw2;
    uint32_t sum1 = 0, sum2 = 0;
    const int samples = 100;

    DisableAllFETs_Dual(); // 모든 FET 비활성화 (정지 상태 보장)

    // Average the raw DMA samples: the filtered ladcValue starts at 0 and is
    // still converging here, and a 16-bit sum overflows (100 x ~2000).
    for (int i = 0; i < samples; i++) {
        sum1 += adc_buf[8];
        sum2 += adc_buf[9];
        HAL_Delay(1); // 1ms 대기
    }

    adc_raw1 = (float)sum1 / (float)samples;
    adc_raw2 = (float)sum2 / (float)samples;

    bemf_offset1 = (adc_raw1 / 4095.0f) * 3.3f / OPAMP_GAIN;
    bemf_offset2 = (adc_raw2 / 4095.0f) * 3.3f / OPAMP_GAIN;

    printf("BEMF offset calibrated: %.1f / %.1f\r\n", bemf_offset1, bemf_offset2);
}

DoubleF_VALUE PWM16::Dual_Motor_GetVelocty(){
#if 1
	const uint8_t samples = 5;
	float motor_spec_voltage = pDataClass->batt.motor_spec_voltage;
	if(motor_spec_voltage < 1.0f)
		motor_spec_voltage = DEFAULT_BATTERY_VOLTAGE;
	const float bemf_to_rpm = MOTOR_MAX_RPM / motor_spec_voltage;
	uint32_t adc_sum1 = 0;
	uint32_t adc_sum2 = 0;

	DisableAllFETs_Dual();
	delay_us(100);
	for(uint8_t i = 0; i < samples; i++) {
		adc_sum1 += adc_buf[8];
		adc_sum2 += adc_buf[9];
		delay_us(10);
	}

	float adc1 = (float)adc_sum1 / (float)samples;
	float adc2 = (float)adc_sum2 / (float)samples;
	// At zero drive PWM the instantaneous ADC can briefly collapse to the
	// midpoint even while the rotor is still coasting. The 10 ms filtered BEMF
	// retains the decaying motor voltage and avoids a false 700 RPM -> 0 jump.
	if(fabsf(pDataClass->inputRaw.target_pwm.f1) <= 0.05f
			&& fabsf(rpm.f1) >= MOTOR_MIN_RPM)
		adc1 = (float)pDataClass->ladcValue[8];
	if(fabsf(pDataClass->inputRaw.target_pwm.f2) <= 0.05f
			&& fabsf(rpm.f2) >= MOTOR_MIN_RPM)
		adc2 = (float)pDataClass->ladcValue[9];
	const float bemf1 = (adc1 / 4095.0f) * 3.3f / OPAMP_GAIN;
	const float bemf2 = (adc2 / 4095.0f) * 3.3f / OPAMP_GAIN;
	static float filtered_rpm1 = 0.0f;
	static float filtered_rpm2 = 0.0f;

	auto calculate_rpm = [bemf_to_rpm](float bemf, float offset,
			float target_pwm, uint8_t fnr,
			float forward_scale, float reverse_scale,
			float previous_rpm) -> float {

		float value = fabsf(bemf - offset) * bemf_to_rpm;
		bool reverse;
		// The actual drive sign (target_pwm) must win: in CAN mode the local
		// FNR lever can sit in reverse while CAN commands forward, which
		// flipped the sign and applied the wrong forward/reverse scale.
		if(target_pwm < -0.01f) reverse = true;
		else if(target_pwm > 0.01f) reverse = false;
		else if(fnr == 2) reverse = true;
		else if(fnr == 1) reverse = false;
		else reverse = (previous_rpm < 0.0f);
		value *= reverse ? reverse_scale : forward_scale;
		// Stopped-state hysteresis: floating-terminal noise reaches ~1.5x
		// MOTOR_MIN_RPM, so leaving the zero state without a drive command
		// requires 2x. Once rotating, track down to MOTOR_MIN_RPM as before.
		float detect_rpm = MOTOR_MIN_RPM;
		if(previous_rpm == 0.0f && fabsf(target_pwm) <= 0.05f)
			detect_rpm = MOTOR_MIN_RPM * 2.0f;
		if(value < detect_rpm) return 0.0f;
		if(value > MOTOR_MAX_RPM) value = MOTOR_MAX_RPM;

		return reverse ? -value : value;
	};

	float rpm1 = calculate_rpm(bemf1, bemf_offset1,
			pDataClass->inputRaw.target_pwm.f1,
			pDataClass->fm2000_gpio.FNR1,
			M1_BEMF_FORWARD_SCALE, M1_BEMF_REVERSE_SCALE,
			filtered_rpm1);
	float rpm2 = calculate_rpm(bemf2, bemf_offset2,
			pDataClass->inputRaw.target_pwm.f2,
			pDataClass->fm2000_gpio.FNR2,
			M2_BEMF_FORWARD_SCALE, M2_BEMF_REVERSE_SCALE,
			filtered_rpm2);

	auto filter_rpm = [](float value, float &filtered, float target_pwm) -> float {
		if(value == 0.0f || filtered * value < 0.0f) {
			filtered = value;
		}
		else {
			const float target_rpm = fabsf(target_pwm) * CONTROL_TARGET_MAX_RPM;
			const float alpha = target_rpm < PID_LOW_SPEED_THRESHOLD
					? RPM_FILTER_ALPHA_LOW_SPEED : RPM_FILTER_ALPHA_NORMAL;
			filtered += alpha * (value - filtered);
		}
		return filtered;
	};

	DoubleF_VALUE measured_rpm;
	measured_rpm.f1 = filter_rpm(rpm1, filtered_rpm1,
			pDataClass->inputRaw.target_pwm.f1);
	measured_rpm.f2 = filter_rpm(rpm2, filtered_rpm2,
			pDataClass->inputRaw.target_pwm.f2);
	// Low-speed BEMF is noisy. Blend it with a PWM motor model for display
	// only; never use this estimate for PID, stall detection or braking.
	auto observe_rpm = [](float raw_rpm, float actual_pwm) -> float {
		const float magnitude = fabsf(raw_rpm);
		float bemf_weight = (magnitude - 300.0f) / 400.0f;
		if(bemf_weight < 0.10f) bemf_weight = 0.10f;
		if(bemf_weight > 1.00f) bemf_weight = 1.00f;
		const float model_rpm = actual_pwm * CONTROL_TARGET_MAX_RPM;
		return bemf_weight * raw_rpm + (1.0f - bemf_weight) * model_rpm;
	};
	observed_rpm.f1 = observe_rpm(measured_rpm.f1, drive_pwm.f1);
	observed_rpm.f2 = observe_rpm(measured_rpm.f2, drive_pwm.f2);
	return measured_rpm;
#else
	uint16_t adc_raw1;
	uint16_t sum1 = 0;
	const int samples = 10;
	DoubleF_VALUE _rpm;
	float BEMF_GAIN = MOTOR_MAX_RPM/24.0f;//pDataClass->batt.motor_spec_voltage;//150.0f; //3600rpm/24V
	float vf1_measured;
	float vBEMF1;
	float m_rpm1,m_rpm2;
	//offset를 정의 기존거(LM2904(2opAMP)-BEMF 수정한거)
	//bemf_offset1=42.51f;
	//bemf_offset2=42.01f;
//(LM2902(4opAMP)-BEMF 수정한거)
	//bemf_offset1=43.82f;
	//bemf_offset2=43.29f;
    // offset 통일 (평균값 사용)
	DisableAllFETs_Dual();
	delay_us(50);

    for (int i = 0; i < samples; i++) {
        sum1 += pDataClass->ladcValue[8];
        sum2 += pDataClass->ladcValue[9];
        HAL_Delay(1); // 1ms 대기
    }

    adc_raw1 = sum1 / samples;

	vf1_measured = (adc_raw1 / 4095.0f) * 3.3f/OPAMP_GAIN;

	vBEMF1 = fabsf(vf1_measured-bemf_offset1);

    m_rpm1=(vBEMF1*BEMF_GAIN);
    m_rpm2=0.0f;

    if(pDataClass->fm2000_gpio.FNR1 == 2
    		|| pDataClass->inputRaw.target_pwm.f1 < -0.01f) {
    	m_rpm1 = -m_rpm1;
    }
    else if(pDataClass->fm2000_gpio.FNR1 != 1
    		&& pDataClass->inputRaw.target_pwm.f1 <= 0.01f) {
    	m_rpm1 = 0.0f;
    }
//ff rr motor rpm differ++
 //   if(m_rpm1<0) m_rpm1 *=(0.915f);
 //   if(m_rpm2<0) m_rpm2 *=(0.915f);
//ff rr motor rpm differ--

    if (fabsf(m_rpm1) < MOTOR_MIN_RPM) m_rpm1 = 0.0f;
    if (fabsf(m_rpm2) < MOTOR_MIN_RPM) m_rpm2 = 0.0f;

    if(m_rpm1>=MOTOR_MAX_RPM) m_rpm1=MOTOR_MAX_RPM;
    if(m_rpm1<=-MOTOR_MAX_RPM) m_rpm1=-MOTOR_MAX_RPM;
    if(m_rpm2>=MOTOR_MAX_RPM) m_rpm2=MOTOR_MAX_RPM;

    // Spike filter: 속도 '급락'만 차단 (BEMF 노이즈), 증가(가속)는 자유 통과
    // 이전값 대비 크기가 40% 이상 갑자기 줄어들면 1샘플 노이즈로 판단
    static float prev_bem_rpm1 = 0.0f, prev_bem_rpm2 = 0.0f;
    float mag1 = fabsf(m_rpm1), prev_mag1 = fabsf(prev_bem_rpm1);
    float mag2 = fabsf(m_rpm2), prev_mag2 = fabsf(prev_bem_rpm2);
    if (m_rpm1 == 0.0f) {
        prev_bem_rpm1 = 0.0f;
    } else if (prev_mag1 > 0.0f && (prev_mag1 - mag1) > prev_mag1 * 0.40f) {
        m_rpm1 = prev_bem_rpm1;  // sudden drop blocked: BEMF noise
    } else {
        prev_bem_rpm1 = m_rpm1;
    }
    if (m_rpm2 == 0.0f) {
        prev_bem_rpm2 = 0.0f;
    } else if (prev_mag2 > 0.0f && (prev_mag2 - mag2) > prev_mag2 * 0.40f) {
        m_rpm2 = prev_bem_rpm2;  // sudden drop blocked: BEMF noise
    } else {
        prev_bem_rpm2 = m_rpm2;
    }

	_rpm.f1=m_rpm1;
	_rpm.f2=m_rpm2;
    //cprintf(C_RED,"@@@@ vf1_measured: %.2f^%.2f rpm(%.0f|%.0f) vBEMF( %.2f| %.2f) vf_measured(%.2f|%.2f) bemf_offset(%.2f|%.2f)\r\n",vf1_measured, vf2_measured, _rpm.f1, _rpm.f2, vBEMF1, vBEMF2, vf1_measured, vf2_measured, bemf_offset1, bemf_offset2);
	return _rpm;
#endif
}


void PWM16::Dual_Motor_set_pwm10(MOTOR_DIRECTION dm_polar, DoubleF_VALUE fSetPwm, uint8_t disable_pid)
{
	int mLeftMotorPwm, mRigjtMotorPwm;
	int pwm1=0,pwm2=0,pwm3=0,pwm4=0,pwm5=0,pwm6=0,pwm7=0,pwm8=0;
	uint8_t dir1, dir2;

	mLeftMotorPwm =  (int)(fminf(fabsf(fSetPwm.f1), 1.0f) * FET_MAX);
    mRigjtMotorPwm = (int)(fminf(fabsf(fSetPwm.f2), 1.0f) * FET_MAX);

    // 부호 → 정방향(0), 역방향(1)
    uint8_t raw_dir1 = (fSetPwm.f1 >= 0.0f) ? 0 : 1;
    uint8_t raw_dir2 = (fSetPwm.f2 >= 0.0f) ? 0 : 1;

    // polar 적용: 물리적 방향 반전 여부
    dir1 = (dm_polar.m1_dir == 1) ? !raw_dir1 : raw_dir1;
    dir2 = (dm_polar.m2_dir == 1) ? !raw_dir2 : raw_dir2;

    // 방향 변경 시 데드타임 확보
    if ((dir1 != prev_direction1) || (dir2 != prev_direction2)) {
        DisableAllFETs_Dual();
        delay_us(5); // μs 단위 데드타임 확보
    }
    prev_direction1 = dir1;
    prev_direction2 = dir2;

//BEMF++
    // Seed the stopped-voltage midpoint before the first BEMF calculation.
    if(mLeftMotorPwm == 0 && bemf_offset1 == 0.0f)
        bemf_offset1 = ((float)adc_buf[8] / 4095.0f) * 3.3f / OPAMP_GAIN;
    if(mRigjtMotorPwm == 0 && bemf_offset2 == 0.0f)
        bemf_offset2 = ((float)adc_buf[9] / 4095.0f) * 3.3f / OPAMP_GAIN;

    // Continue measuring BEMF while the motor is coasting after drive PWM
    // reaches zero. The measured speed is also used by the stop brake logic.
    rpm = Dual_Motor_GetVelocty();

    // Each stopped motor tracks its own BEMF midpoint independently.
    if(mLeftMotorPwm == 0 && fabsf(rpm.f1) < MOTOR_MIN_RPM) {
		float offset = ((float)adc_buf[8] / 4095.0f) * 3.3f / OPAMP_GAIN;
		if(bemf_offset1 == 0.0f) bemf_offset1 = offset;
		else bemf_offset1 += 0.05f * (offset - bemf_offset1);
		rpm.f1 = 0.0f;
	}
    if(mRigjtMotorPwm == 0 && fabsf(rpm.f2) < MOTOR_MIN_RPM) {
        float offset = ((float)adc_buf[9] / 4095.0f) * 3.3f / OPAMP_GAIN;
        if(bemf_offset2 == 0.0f) bemf_offset2 = offset;
        else bemf_offset2 += 0.05f * (offset - bemf_offset2);
        rpm.f2 = 0.0f;
    }
    // Recovery from a corrupted midpoint: a wrong offset keeps the computed
    // rpm above MOTOR_MIN_RPM forever, which blocks the rpm-gated tracking
    // above (rpm pegs at MOTOR_MAX_RPM). After 3 s of continuous zero drive
    // the motor is mechanically stopped, so re-track regardless of rpm.
    static uint16_t zero_pwm_ticks1 = 0, zero_pwm_ticks2 = 0;
    if(mLeftMotorPwm == 0) { if(zero_pwm_ticks1 < 60000) zero_pwm_ticks1++; }
    else zero_pwm_ticks1 = 0;
    if(mRigjtMotorPwm == 0) { if(zero_pwm_ticks2 < 60000) zero_pwm_ticks2++; }
    else zero_pwm_ticks2 = 0;
    if(zero_pwm_ticks1 > 300 && fabsf(rpm.f1) >= MOTOR_MIN_RPM) {
        float offset = ((float)adc_buf[8] / 4095.0f) * 3.3f / OPAMP_GAIN;
        bemf_offset1 += 0.05f * (offset - bemf_offset1);
    }
    if(zero_pwm_ticks2 > 300 && fabsf(rpm.f2) >= MOTOR_MIN_RPM) {
        float offset = ((float)adc_buf[9] / 4095.0f) * 3.3f / OPAMP_GAIN;
        bemf_offset2 += 0.05f * (offset - bemf_offset2);
    }
//BEMF--

//    mRigjtMotorPwm=FET_MAX;
//    mLeftMotorPwm=FET_MAX;//500;//0;
//PWM=0 -> Active Freewheeling (BRAKE 방지)
//	if(mLeftMotorPwm == 0 && mRigjtMotorPwm == 0){
//		//DisableAllFETs_Dual();
//		pwm1=pwm2=pwm3=pwm4=0;
//		return;
//	}
//M1
	if(mLeftMotorPwm > 0)
	{
		if(dir1==0){
			pwm1=pwm2=mLeftMotorPwm;
			pwm2=pwm1+PWM_DAED_TIME;
		}
		else {
			pwm3=pwm4=mLeftMotorPwm;
			pwm4=pwm3+PWM_DAED_TIME;
		}
	}
	else{pwm2=pwm4=0;pwm1=pwm3=0;}
//M2
	if(mRigjtMotorPwm > 0)
	{
		if(dir2==0){
			pwm5=pwm6=mRigjtMotorPwm;
			pwm6=pwm5+PWM_DAED_TIME;
		}
		else {
			pwm7=pwm8=mRigjtMotorPwm;
			pwm8=pwm7+PWM_DAED_TIME;
		}
	}
	else{pwm6=pwm8=0;pwm5=pwm7=0;}
	//if(pwm==0)pwm1=pwm2=pwm3=pwm4=0;
	htim3.Instance->CCR1=pwm1;
	htim3.Instance->CCR2=pwm2;
	htim3.Instance->CCR3=pwm3;
	htim3.Instance->CCR4=pwm4;

	htim2.Instance->CCR1=pwm5;
	htim2.Instance->CCR2=pwm6;
	htim2.Instance->CCR3=pwm7;
	htim2.Instance->CCR4=pwm8;
}

void PWM16::ApplyShortBrakeDuty(uint8_t brake_m1, float duty_m1,
		uint8_t brake_m2, float duty_m2)
{
	if(duty_m1 < 0.0f) duty_m1 = 0.0f;
	if(duty_m1 > 1.0f) duty_m1 = 1.0f;
	if(duty_m2 < 0.0f) duty_m2 = 0.0f;
	if(duty_m2 > 1.0f) duty_m2 = 1.0f;

	/*
	 * High-side outputs remain OFF. The low-side channels use LOW polarity:
	 * CCR=1024 is coast and CCR=0 is full low-side short braking.
	 */
	if(brake_m1) {
		uint16_t ccr_m1 = (uint16_t)((1.0f - duty_m1)
				* (float)SHORT_BRAKE_PWM_PERIOD);
		htim3.Instance->CCR1 = 0;
		htim3.Instance->CCR2 = ccr_m1;
		htim3.Instance->CCR3 = 0;
		htim3.Instance->CCR4 = ccr_m1;
	}
	if(brake_m2) {
		uint16_t ccr_m2 = (uint16_t)((1.0f - duty_m2)
				* (float)SHORT_BRAKE_PWM_PERIOD);
		htim2.Instance->CCR1 = 0;
		htim2.Instance->CCR2 = ccr_m2;
		htim2.Instance->CCR3 = 0;
		htim2.Instance->CCR4 = ccr_m2;
	}
}

void PWM16::ResetShortBrakeControl()
{
	short_brake_base_duty_m1 = 0.0f;
	short_brake_base_duty_m2 = 0.0f;
	short_brake_duty_m1 = 0.0f;
	short_brake_duty_m2 = 0.0f;
	short_brake_applied_duty_m1 = 0.0f;
	short_brake_applied_duty_m2 = 0.0f;
	short_brake_vdc_reference_m1 = 0.0f;
	short_brake_vdc_reference_m2 = 0.0f;
	short_brake_entry_pwm_m1 = 0.0f;
	short_brake_entry_pwm_m2 = 0.0f;
	short_brake_active_m1 = 0;
	short_brake_active_m2 = 0;
	short_brake_vdc_full_m1 = 0;
	short_brake_vdc_full_m2 = 0;
	short_brake_pulse_count_m1 = 0;
	short_brake_pulse_count_m2 = 0;
	short_brake_vdc_filter = 0.0f;
	short_brake_filter_active = 0;
}

DoubleF_VALUE PWM16::GetShortBrakeDuty()
{
	DoubleF_VALUE duty;
	duty.f1 = short_brake_applied_duty_m1;
	duty.f2 = short_brake_applied_duty_m2;
	return duty;
}

float PWM16::UpdateOneShortBrake_1ms(uint8_t brake_request,
		float dc_link_voltage, float source_pwm_abs, float target_pwm_abs,
		float &base_duty, float &output_duty,
		float &vdc_reference, float &entry_pwm, uint8_t &active)
{
	float system_reference = ShortBrakeReference(dc_link_voltage);
	float vdc_warn = ShortBrakeThreshold(system_reference, SHORT_BRAKE_VDC_WARN_RATIO);
	float vdc_protect = ShortBrakeThreshold(system_reference, SHORT_BRAKE_VDC_PROTECT_RATIO);
	float vdc_hard = ShortBrakeThreshold(system_reference, SHORT_BRAKE_VDC_HARD_RATIO);
	if(!brake_request) {
		base_duty = 0.0f;
		output_duty = 0.0f;
		vdc_reference = 0.0f;
		entry_pwm = 0.0f;
		active = 0;
		return 0.0f;
	}

	if(!active) {
		active = 1;
		base_duty = SHORT_BRAKE_INITIAL_DUTY;
		output_duty = SHORT_BRAKE_INITIAL_DUTY;
		vdc_reference = dc_link_voltage;
		entry_pwm = target_pwm_abs;
		if(entry_pwm < 0.05f) entry_pwm = 0.05f;
	}

	// Normal brake feel follows the existing deceleration trajectory instead
	// of an independent time ramp.
	float decel_progress = 1.0f - (target_pwm_abs / entry_pwm);
	if(decel_progress < SHORT_BRAKE_INITIAL_DUTY) {
		decel_progress = SHORT_BRAKE_INITIAL_DUTY;
	}
	if(decel_progress > 1.0f) decel_progress = 1.0f;

	// Taper normal braking near zero speed/command. The overvoltage pulse
	// protection below remains available, but the normal deceleration ramp
	// must not approach a continuous full short at the final stop.
	if(target_pwm_abs < SHORT_BRAKE_LOW_SPEED_PWM) {
		float low_speed_scale =
				target_pwm_abs / SHORT_BRAKE_LOW_SPEED_PWM;
		decel_progress *= low_speed_scale;
		if(decel_progress < SHORT_BRAKE_INITIAL_DUTY)
			decel_progress = SHORT_BRAKE_INITIAL_DUTY;
	}
	base_duty = decel_progress;

	float target_duty = base_duty;

	// Feed-forward braking acts before the DC-link rises. A large difference
	// between the still-ramped motor target and the newly requested source
	// means that the upper controller has requested rapid deceleration.
	float command_drop = target_pwm_abs - source_pwm_abs;
	if(command_drop < 0.0f) command_drop = 0.0f;
	float feedforward_duty = (entry_pwm > 0.05f)
			? (command_drop / entry_pwm) * SHORT_BRAKE_FEEDFORWARD_GAIN
			: 0.0f;
	if(feedforward_duty > SHORT_BRAKE_FEEDFORWARD_MAX)
		feedforward_duty = SHORT_BRAKE_FEEDFORWARD_MAX;
	if(feedforward_duty > target_duty) target_duty = feedforward_duty;

	float vdc_rise = dc_link_voltage - vdc_reference;
	if(vdc_rise < 0.0f) vdc_rise = 0.0f;
	float vdc_rise_ratio = (vdc_reference > 1.0f)
			? (vdc_rise / vdc_reference) : 0.0f;

	// Only the absolute hardware limit may bypass the duty slew. A relative
	// rise can be calculated from a temporarily sagged reference voltage; an
	// immediate full short in that case causes a severe torque step.
	if(dc_link_voltage >= vdc_hard) {
		output_duty = 1.0f;
		return output_duty;
	}

	if(vdc_rise_ratio >= SHORT_BRAKE_VDC_RISE_START_RATIO) {
		// Relative DC-link feedback changes the short-brake PWM pulse width:
		// about +1 % -> 20 %, +2.5 % -> 50 %, +5 % -> 100 %. Unlike the
		// old 1 ms full-short/coast pulse, this uses the motor PWM period and
		// reaches the target through a duty slew to avoid a torque step.
		float ratio = vdc_rise_ratio / SHORT_BRAKE_VDC_MAX_RATIO;
		if(ratio > 1.0f) ratio = 1.0f;
		float ratio_duty = ratio * SHORT_BRAKE_VDC_RATIO_DUTY_MAX;
		if(ratio_duty > target_duty) target_duty = ratio_duty;
	}

	// Absolute-voltage protection is staged so normal DC-link feedback does
	// not turn a smooth stop into an immediate full short.
	if(dc_link_voltage >= vdc_protect && target_duty < 0.75f) {
		target_duty = 0.75f;
	}
	else if(dc_link_voltage >= vdc_warn && target_duty < 0.45f) {
		target_duty = 0.45f;
	}
	if(target_duty > 1.0f) target_duty = 1.0f;

	if(target_duty < 0.0f) target_duty = 0.0f;

	float rise_step = SHORT_BRAKE_RAMP_PER_MS;
	if(dc_link_voltage >= vdc_protect
			|| vdc_rise_ratio >= 0.08f) {
		rise_step = SHORT_BRAKE_DUTY_FAST_STEP;
	}
	else if(dc_link_voltage >= vdc_warn
			|| vdc_rise_ratio >= 0.05f) {
		rise_step = SHORT_BRAKE_DUTY_WARN_STEP;
	}
	else if(vdc_rise_ratio >= SHORT_BRAKE_VDC_RISE_START_RATIO) {
		rise_step = SHORT_BRAKE_DUTY_VDC_STEP;
	}
	else if(feedforward_duty > output_duty) {
		// 0 -> 30 % in approximately 100 ms.
		rise_step = SHORT_BRAKE_FEEDFORWARD_STEP;
	}
	if(output_duty < target_duty) {
		if(vdc_rise_ratio >= SHORT_BRAKE_VDC_RISE_START_RATIO) {
			// DC-link energy rises much faster than the previous duty slew.
			// Apply the calculated PWM pulse width immediately; this is still
			// PWM short braking, not a millisecond full-short/coast toggle.
			output_duty = target_duty;
		}
		else {
			output_duty += rise_step;
			if(output_duty > target_duty) output_duty = target_duty;
		}
	}
	else if(output_duty > target_duty) {
		// Release braking slowly to prevent DC-link feedback hunting and the
		// resulting audible/mechanical torque step.
		float release_step = (target_pwm_abs < SHORT_BRAKE_LOW_SPEED_PWM)
				? SHORT_BRAKE_LOW_SPEED_RELEASE
				: SHORT_BRAKE_DUTY_RELEASE_STEP;
		output_duty -= release_step;
		if(output_duty < target_duty) output_duty = target_duty;
	}

	return output_duty;
}

void PWM16::UpdateShortBrakeControl_1ms(uint8_t brake_m1, uint8_t brake_m2,
		float source_pwm_abs_m1, float source_pwm_abs_m2,
		float target_pwm_abs_m1, float target_pwm_abs_m2,
		float dc_link_voltage)
{
	float system_reference = ShortBrakeReference(dc_link_voltage);
	float vdc_hard = ShortBrakeThreshold(system_reference, SHORT_BRAKE_VDC_HARD_RATIO);
	if(!short_brake_filter_active) {
		short_brake_filter_active = 1;
		short_brake_vdc_filter = dc_link_voltage;
	}
	else {
		// Common measurements: about 4 ms switching-noise filter.
		short_brake_vdc_filter +=
				0.25f * (dc_link_voltage - short_brake_vdc_filter);
	}

	// Raw ADC overvoltage bypasses the filter delay.
	if(dc_link_voltage >= vdc_hard) {
		short_brake_vdc_filter = dc_link_voltage;
	}

	if(!brake_m1 && !brake_m2) {
		// Keep the common DC-link filter alive so each motor captures the
		// pre-braking voltage, but reset the two independent brake states.
		short_brake_base_duty_m1 = 0.0f;
		short_brake_base_duty_m2 = 0.0f;
		short_brake_duty_m1 = 0.0f;
		short_brake_duty_m2 = 0.0f;
		short_brake_applied_duty_m1 = 0.0f;
		short_brake_applied_duty_m2 = 0.0f;
		short_brake_vdc_reference_m1 = 0.0f;
		short_brake_vdc_reference_m2 = 0.0f;
		short_brake_entry_pwm_m1 = 0.0f;
		short_brake_entry_pwm_m2 = 0.0f;
		short_brake_active_m1 = 0;
		short_brake_active_m2 = 0;
		short_brake_vdc_full_m1 = 0;
		short_brake_vdc_full_m2 = 0;
		short_brake_pulse_count_m1 = 0;
		short_brake_pulse_count_m2 = 0;
		return;
	}

	float duty_m1 = UpdateOneShortBrake_1ms(brake_m1,
			short_brake_vdc_filter, source_pwm_abs_m1, target_pwm_abs_m1,
			short_brake_base_duty_m1, short_brake_duty_m1,
			short_brake_vdc_reference_m1, short_brake_entry_pwm_m1,
			short_brake_active_m1);
	float duty_m2 = UpdateOneShortBrake_1ms(brake_m2,
			short_brake_vdc_filter, source_pwm_abs_m2, target_pwm_abs_m2,
			short_brake_base_duty_m2, short_brake_duty_m2,
			short_brake_vdc_reference_m2, short_brake_entry_pwm_m2,
			short_brake_active_m2);

	// Relative DC-link rise is handled by PWM duty feedback in
	// UpdateOneShortBrake_1ms(). Do not alternate full-short/coast at 1 ms
	// intervals: that binary torque step is audible and causes a mechanical
	// knock. Only the absolute 58 V hardware-protection limit may bypass the
	// duty ramp and request a continuous full short.
	if(brake_m1 && short_brake_vdc_reference_m1 > 1.0f) {
		if(dc_link_voltage >= vdc_hard) {
			short_brake_vdc_full_m1 = 1;
			short_brake_pulse_count_m1 = 0;
			duty_m1 = 1.0f;
		}
		else {
			short_brake_vdc_full_m1 = 0;
			short_brake_pulse_count_m1 = 0;
		}
	}
	else {
		short_brake_vdc_full_m1 = 0;
		short_brake_pulse_count_m1 = 0;
	}

	if(brake_m2 && short_brake_vdc_reference_m2 > 1.0f) {
		if(dc_link_voltage >= vdc_hard) {
			short_brake_vdc_full_m2 = 1;
			short_brake_pulse_count_m2 = 0;
			duty_m2 = 1.0f;
		}
		else {
			short_brake_vdc_full_m2 = 0;
			short_brake_pulse_count_m2 = 0;
		}
	}
	else {
		short_brake_vdc_full_m2 = 0;
		short_brake_pulse_count_m2 = 0;
	}

	short_brake_applied_duty_m1 = brake_m1 ? duty_m1 : 0.0f;
	short_brake_applied_duty_m2 = brake_m2 ? duty_m2 : 0.0f;
	ApplyShortBrakeDuty(brake_m1, short_brake_applied_duty_m1,
			brake_m2, short_brake_applied_duty_m2);
}

DoubleF_VALUE PWM16::pidCalibration(uint8_t disable_pid, DoubleF_VALUE input, DoubleF_VALUE pid_pwm)
{
    DoubleF_VALUE result;
    
    if (disable_pid) {
        result = input;
    } else {
        result.f1 = input.f1 + pid_pwm.f1;
        result.f2 = input.f2 + pid_pwm.f2;
        
        // PWM 제한
        if (result.f1 > 1.0f) result.f1 = 1.0f;
        if (result.f1 < -1.0f) result.f1 = -1.0f;
        if (result.f2 > 1.0f) result.f2 = 1.0f;
        if (result.f2 < -1.0f) result.f2 = -1.0f;
    }
    
    return result;
}

void PWM16::Update_PWM(uint8_t disable_pid, float target_pwm1, float target_pwm2)
{
	MOTOR_DIRECTION md_polarity;
	DoubleF_VALUE set_pwm;
    DoubleF_VALUE raw_target_pwm;
    DoubleF_VALUE out_pwm;

    md_polarity.m1_dir = pDataClass->motor1_polarity;
    md_polarity.m2_dir = pDataClass->motor2_polarity;
	brake_delay_timeout_init=pDataClass->brake_delay/10;

	// NOTE: CheckBrakeState는 ten_millisec_routine()에서 Update_PWM 호출 후 별도 호출됨
	// 이곳에서 중복 호출 시 brake 타이머가 2배 속도로 감소하는 버그 발생 → 제거

	raw_target_pwm.f1 = target_pwm1;  // -1.0~1.0
	raw_target_pwm.f2 = target_pwm2;
	
	// 스로틀 정지 구간에서는 PID 적분 동결 (Anti-windup)
	if (stop_throttle) {
		integral1 = 0;
		integral2 = 0;
	}

    float current_rpm1  = pDataClass->inputRaw.rpm.f1;
    float current_rpm2  = pDataClass->inputRaw.rpm.f2;

    // 보호: 비정상 rpm 값 방지
    if (isnan(current_rpm1) || fabsf(current_rpm1) > 100000.0f) current_rpm1 = 0.0f;
    if (isnan(current_rpm2) || fabsf(current_rpm2) > 100000.0f) current_rpm2 = 0.0f;

    // Convert the operator command to the measured motor-speed range.
    // MOTOR_MAX_RPM is also used by the BEMF conversion, so changing it here
    // would change the measured RPM itself. Keep a separate control target.
    target_rpm.f1 = raw_target_pwm.f1 * CONTROL_TARGET_MAX_RPM;
    target_rpm.f2 = raw_target_pwm.f2 * CONTROL_TARGET_MAX_RPM;

    // Dual_Motor_GetVelocty() already returns signed RPM:
    // forward is positive and reverse is negative. Applying motor polarity
    // again here reverses the measured sign twice and makes reverse-direction
    // RPM error incorrect.
    error_rpm.f1 = target_rpm.f1 - current_rpm1;
    error_rpm.f2 = target_rpm.f2 - current_rpm2;
    
    // BEMF speed is unreliable near standstill.  Keep feed-forward/S-curve
    // control below 500 rpm and enter closed loop only after both target and
    // measured speed exceed 600 rpm.  Direction reversal also re-arms PID.
    auto update_pid_gate = [](float target, float measured, uint8_t &active,
			uint8_t &entry_ticks) -> uint8_t {
		const uint8_t same_direction = (target * measured) > 0.0f;
		if(active) {
			if(!same_direction || fabsf(target) < PID_LOW_SPEED_THRESHOLD
					|| fabsf(measured) < PID_LOW_SPEED_THRESHOLD) {
				active = 0;
				entry_ticks = 0;
			}
		}
		else if(same_direction && fabsf(target) >= PID_LOW_SPEED_EXIT_THRESHOLD
				&& fabsf(measured) >= PID_LOW_SPEED_EXIT_THRESHOLD) {
			active = 1;
			entry_ticks = PID_ENTRY_LIMIT_TICKS;
		}
		return active;
	};

    const uint8_t pid_gate_m1 = update_pid_gate(target_rpm.f1, current_rpm1,
			pid_active_m1, pid_entry_ticks_m1);
    const uint8_t pid_gate_m2 = update_pid_gate(target_rpm.f2, current_rpm2,
			pid_active_m2, pid_entry_ticks_m2);

    // Stop, explicit disable, and low-speed operation reset the integrator.
    if (disable_pid || stop_throttle) {
        pid_pwm.f1 = 0.0f;
        pid_pwm.f2 = 0.0f;
		integral1 = integral2 = 0.0f;
		pid_active_m1 = pid_active_m2 = 0;
		pid_entry_ticks_m1 = pid_entry_ticks_m2 = 0;
    }
    else {
		if(pid_gate_m1) pid_pwm.f1 = PID_Compute1(error_rpm.f1);
		else { pid_pwm.f1 = 0.0f; integral1 = 0.0f; prev_error1 = error_rpm.f1; }
		if(pid_gate_m2) pid_pwm.f2 = PID_Compute2(error_rpm.f2);
		else { pid_pwm.f2 = 0.0f; integral2 = 0.0f; prev_error2 = error_rpm.f2; }
    }

    // Limit the correction to +/-5% for the first 200 ms after PID entry,
    // then allow the existing +/-20% normal correction range.
    const float pid_limit1 = pid_entry_ticks_m1
			? PID_ENTRY_CORRECTION_LIMIT : PID_NORMAL_CORRECTION_LIMIT;
    const float pid_limit2 = pid_entry_ticks_m2
			? PID_ENTRY_CORRECTION_LIMIT : PID_NORMAL_CORRECTION_LIMIT;
    pid_pwm.f1 = fmaxf(fminf(pid_pwm.f1, pid_limit1), -pid_limit1);
    pid_pwm.f2 = fmaxf(fminf(pid_pwm.f2, pid_limit2), -pid_limit2);
	if(pid_active_m1 && pid_entry_ticks_m1) pid_entry_ticks_m1--;
	if(pid_active_m2 && pid_entry_ticks_m2) pid_entry_ticks_m2--;

    // [전속도 PID] 저속/고속 구분 없이 스로틀 입력이 있으면 항상 PID 보정 적용
    // - 고속 부하 보상: 스로틀 80%에서 부하로 RPM 저하 시 자동 보정
    // - 스로틀 100% 포화 구간은 클램프로 인해 보정 불가 (물리적 한계)
    // - stop_throttle 구간은 적분 windup 방지를 위해 비활성

#if 1
    // PID 출력 보정 적용: 전속도 구간 (스로틀 있을 때 항상)
    if(disable_pid || stop_throttle){
        out_pwm = raw_target_pwm;
    }
    else
    {
		 out_pwm=pidCalibration(disable_pid, raw_target_pwm, pid_pwm);
    }
#else
    out_pwm=raw_target_pwm;
#endif

    // 데드존 적용
    if (fabsf(out_pwm.f1) < DEADZONE_THRESHOLD) out_pwm.f1 = 0.0f;
    if (fabsf(out_pwm.f2) < DEADZONE_THRESHOLD) out_pwm.f2 = 0.0f;

    // 범위 제한
    out_pwm.f1 = fmaxf(fminf(out_pwm.f1, 1.0f), -1.0f);
    out_pwm.f2 = fmaxf(fminf(out_pwm.f2, 1.0f), -1.0f);

    set_pwm.f1 = out_pwm.f1;
    set_pwm.f2 = out_pwm.f2;

   // printf("set_pwm[%.2f^%.2f] out_pwm[%.2f^%.2f] pid_pwm[%.2f^%.2f]\r\n",set_pwm.f1, set_pwm.f2, out_pwm.f1, out_pwm.f2, pid_pwm.f1, pid_pwm.f2);

    // PRELOAD is the only state allowed to drive against the applied
    // mechanical brake. Its command is limited before reaching this method.
    if(brake_on_flag && emb_start_state != EMB_PRELOAD) {
        set_pwm.f1 = set_pwm.f2 = 0;
        integral1 = integral2 = 0;
    }

    // Per-motor drive/brake interlock. The 1 ms short-brake controller owns
    // the bridge while braking; the 10 ms drive update must never restore a
    // non-zero drive PWM on that motor.
    if(short_brake_active_m1) set_pwm.f1 = 0.0f;
    if(short_brake_active_m2) set_pwm.f2 = 0.0f;

    // Store the actual drive command for control diagnostics.
    drive_pwm = set_pwm;

    //printf("POLARITY: m1_dir[%d] m2_dir[%d] jenhujin[%d]\r\n",  md_polarity.m1_dir, md_polarity.m2_dir, pDataClass->ctl_pwm.io.toggle.jenhujin);
    // 듀얼 모터 PWM 출력 적용
    Dual_Motor_set_pwm10(md_polarity, set_pwm, disable_pid);

    // Dual_Motor_set_pwm10() writes all CCR registers. Restore the current
    // short-brake waveform immediately instead of waiting for the next
    // 1 ms control tick.
    if(short_brake_active_m1 || short_brake_active_m2) {
    	ApplyShortBrakeDuty(short_brake_active_m1, short_brake_applied_duty_m1,
    			short_brake_active_m2, short_brake_applied_duty_m2);
    }

#if 0
    printf("@TogStop[%d^%d] curRPM[%+05d^%+05d] targetPWM[%+03.2f^%+03.2f] targetRPM[%+05d^%+05d] errRPM[%+05d^%+05d] pidPWM[%+03.2f^%+03.2f] setPWM[%+03.2f^%+03.2f] BK_delay[%02d] stopRPM[%d] stopTHRO[%d]  ovCNT[%02d] avrRPM[%+05d^%+05d] bakF[%d]\r\n",
			pDataClass->inputRaw.io.toggle, pDataClass->inputRaw.io.toggle.t_sw,
			(int)rpm.f1, (int)rpm.f2,raw_target_pwm.f1, raw_target_pwm.f2,
			(int)target_rpm.f1, (int)target_rpm.f2,
			(int)error_rpm.f1, (int)error_rpm.f2,
			pid_pwm.f1, pid_pwm.f2,	set_pwm.f1, set_pwm.f2,
			brake_delay_timeout, stop_rpm_flag, stop_throttle, over_cnt,(int)avr_rpm.f1, (int)avr_rpm.f2, brake_on_flag);
#endif
}

float PWM16::LimitElectromagneticBrakeStartPwm(float requested_pwm) const
{
	if(!IsElectromagneticBrakeStarting()) return 0.0f;

	float pwm_limit = emb_preload_overcurrent
			? EMB_PRELOAD_DERATED_PWM : EMB_PRELOAD_PWM;
	if(emb_start_state == EMB_PRELOAD) {
		// Build holding torque smoothly instead of applying a 15% step.
		float phase = (float)emb_start_ticks / (float)EMB_PRELOAD_TICKS;
		if(phase < 0.0f) phase = 0.0f;
		if(phase > 1.0f) phase = 1.0f;
		pwm_limit *= phase;
	}
	return copysignf(fminf(fabsf(requested_pwm), pwm_limit), requested_pwm);
}

void PWM16::RequestDriveEnable(uint8_t enable, float measured_current_a)
{
	if(!enable) return;
	if(brake_on_flag && emb_start_state != EMB_PRELOAD) {
		emb_start_state = EMB_APPLIED;
		emb_start_ticks = 0;
	}

	// Do not reset the start timer on a brief current peak; that caused an
	// unpredictable delayed launch. Derate only the preload command instead.
	emb_preload_overcurrent =
			(measured_current_a > EMB_PRELOAD_MAX_CURRENT_A) ? 1U : 0U;

	switch(emb_start_state) {
	case EMB_APPLIED:
		brake_on_flag = 1;
		emb_start_state = EMB_PRELOAD;
		emb_start_ticks = 0;
		break;
	case EMB_PRELOAD:
		if(++emb_start_ticks >= EMB_PRELOAD_TICKS) {
			brake_on_flag = 0;
			emb_start_state = EMB_RELEASE_SETTLE;
			emb_start_ticks = 0;
		}
		break;
	case EMB_RELEASE_SETTLE:
		if(++emb_start_ticks >= EMB_RELEASE_SETTLE_TICKS) {
			emb_start_state = EMB_RUN;
			emb_start_ticks = 0;
		}
		break;
	case EMB_RUN:
	default:
		break;
	}

	brake_delay_timeout = brake_delay_timeout_init;
	m1_stop_pwm = m2_stop_pwm = 0;
	m1_stop_rpm = m2_stop_rpm = 0;
	m1_over_cnt = m2_over_cnt = EMB_STOP_CONFIRM_TICKS;
	stop_throttle = 0;
	if(pDataClass != 0) pDataClass->sysFlag.stop_throttle = 0;
}

void PWM16::CheckBrakeState(float stop_pwm1, float stop_pwm2)
{
	// 1. 각 모터별 PWM 정지 감지 (독립적)
    if (m1_stop_pwm == 0) {
        if (fabsf(stop_pwm1) <= 0.05f) {
        	m1_stop_pwm = 1;
        	m1_value = rpm.f1;
            //printf("[M1 THROTTLE OFF]\r\n");
        }
    }
    
    if (m2_stop_pwm == 0) {
        if (fabsf(stop_pwm2) <= 0.05f) {
        	m2_stop_pwm = 1;
        	m2_value = rpm.f2;
            //printf("[M2 THROTTLE OFF]\r\n");
        }
    }

    // 필터 업데이트 (항상 수행)
	avr_rpm.f1=get_m1_filter(rpm.f1);
	avr_rpm.f2=get_m2_filter(rpm.f2);

    // 2. 각 모터별 RPM 감속 완료 감지 (독립적)
    if(!m1_stop_rpm && m1_stop_pwm){
		if(fabsf(avr_rpm.f1) < EMB_APPLY_MAX_RPM) {
			if(m1_over_cnt)m1_over_cnt--;
		}
		else m1_over_cnt=EMB_STOP_CONFIRM_TICKS;

    	if(m1_over_cnt==0) m1_stop_rpm = 1;
   }
   
    if(!m2_stop_rpm && m2_stop_pwm){
		if(fabsf(avr_rpm.f2) < EMB_APPLY_MAX_RPM) {
			if(m2_over_cnt)m2_over_cnt--;
		}
		else m2_over_cnt=EMB_STOP_CONFIRM_TICKS;

    	if(m2_over_cnt==0) m2_stop_rpm = 1;
   }

    // 3. 양쪽 모두 정지 조건 만족 시 브레이크 활성화
    stop_throttle = (m1_stop_pwm && m2_stop_pwm) ? 1 : 0;
    stop_rpm_flag = (m1_stop_rpm && m2_stop_rpm) ? 1 : 0;
    
    // dataclass의 stop_throttle과 동기화
    pDataClass->sysFlag.stop_throttle = stop_throttle;
    
    // 브레이크 딜레이 타이머
    if (stop_throttle == 1) {
        if(stop_rpm_flag) {
        	if (brake_delay_timeout > 0) {
        		brake_delay_timeout--;
        	}
        	else {
        		brake_on_flag = 1;
			emb_start_state = EMB_APPLIED;
			emb_start_ticks = 0;
			emb_preload_overcurrent = 0;
        	}
        }
        else {
        	brake_delay_timeout = brake_delay_timeout_init;
        }
    }

    // 4. 재가동 감지 - 히스테리시스 적용
    if (fabsf(stop_pwm1) > STOP_INPUT_PWM) {
    	m1_stop_pwm = 0;
    	m1_stop_rpm = 0;
		m1_over_cnt = EMB_STOP_CONFIRM_TICKS;
    }
    
    if (fabsf(stop_pwm2) > STOP_INPUT_PWM) {
    	m2_stop_pwm = 0;
    	m2_stop_rpm = 0;
		m2_over_cnt = EMB_STOP_CONFIRM_TICKS;
    }
    
	// Starting is owned exclusively by RequestDriveEnable(). Do not release
	// the relay here; this function only detects stopping and applies it.
    if (fabsf(stop_pwm1) > STOP_INPUT_PWM || fabsf(stop_pwm2) > STOP_INPUT_PWM) {
    	brake_delay_timeout=brake_delay_timeout_init;
    	integral1=integral2=0;
    }
}

float PWM16::get_m1_filter(float value)
{
  m1_value=(m1_value*(1-FILTER_SENSITIVITY))+(value*FILTER_SENSITIVITY);
  return m1_value;
}

float PWM16::get_m2_filter(float value)
{
  m2_value=(m2_value*(1-FILTER_SENSITIVITY))+(value*FILTER_SENSITIVITY);
  return m2_value;
}
