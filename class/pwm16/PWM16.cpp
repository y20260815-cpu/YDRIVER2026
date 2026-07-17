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
#define PID_LOW_SPEED_EXIT_THRESHOLD 600.0f  // PID 탈출 임계값 (진입 500 / 탈출 600 히스테리시스)

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
	rpm.f1=0.0f;
	rpm.f2=0.0f;  // 재부팅 직후 쓰레기값 방지
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
    uint16_t adc_raw1, adc_raw2;
    uint16_t sum1 = 0, sum2 = 0;
    const int samples = 100;

    DisableAllFETs_Dual(); // 모든 FET 비활성화 (정지 상태 보장)

    for (int i = 0; i < samples; i++) {
        pDataClass->Get_AdcData();
        sum1 += pDataClass->ladcValue[8];
        sum2 += pDataClass->ladcValue[9];
        HAL_Delay(1); // 1ms 대기
    }

    adc_raw1 = sum1 / samples;
    adc_raw2 = sum2 / samples;

    bemf_offset1 = (adc_raw1 / 4095.0f) * 3.3f / OPAMP_GAIN;
    bemf_offset2 = (adc_raw2 / 4095.0f) * 3.3f / OPAMP_GAIN;

    //printf("BEMF Offset Calibrated: bemf_offset1 = %.3f, bemf_offset2 = %.3f\r\n", bemf_offset1, bemf_offset2);
}

DoubleF_VALUE PWM16::Dual_Motor_GetVelocty(){
	uint16_t adc_raw1, adc_raw2;
	uint16_t sum1 = 0, sum2 = 0;
	const int samples = 10;
	DoubleF_VALUE _rpm;
	float BEMF_GAIN = MOTOR_MAX_RPM/24.0f;//pDataClass->batt.motor_spec_voltage;//150.0f; //3600rpm/24V
	float vf1_measured, vf2_measured;
	float vBEMF1, vBEMF2;
	float m_rpm1,m_rpm2;
	//offset를 정의 기존거(LM2904(2opAMP)-BEMF 수정한거)
	//bemf_offset1=42.51f;
	//bemf_offset2=42.01f;
//(LM2902(4opAMP)-BEMF 수정한거)
	//bemf_offset1=43.82f;
	//bemf_offset2=43.29f;
    // offset 통일 (평균값 사용)
    bemf_offset1 = 38.35f;
    bemf_offset2 = 38.72f;

	DisableAllFETs_Dual();
	delay_us(5);//	waiting for stable (~1ms real: delay_us*0.5us@72MHz) //2000

    for (int i = 0; i < samples; i++) {
        pDataClass->Get_AdcData();
        sum1 += pDataClass->ladcValue[8];
        sum2 += pDataClass->ladcValue[9];
 //       HAL_Delay(1); // 1ms 대기
    }

    adc_raw1 = sum1 / samples;
    adc_raw2 = sum2 / samples;

	vf1_measured = (adc_raw1 / 4095.0f) * 3.3f/OPAMP_GAIN;
	vf2_measured = (adc_raw2 / 4095.0f) * 3.3f/OPAMP_GAIN;

	vBEMF1 =vf1_measured-bemf_offset1;
    vBEMF2 =vf2_measured-bemf_offset2;
//too much differ Motor direction just calcuration
    if(vBEMF1>0)vBEMF1*=2.91f;
    if(vBEMF2>0)vBEMF2*=2.91f;

    m_rpm1=(vBEMF1*BEMF_GAIN);
    m_rpm2=(vBEMF2*BEMF_GAIN);
//ff rr motor rpm differ++
 //   if(m_rpm1<0) m_rpm1 *=(0.915f);
 //   if(m_rpm2<0) m_rpm2 *=(0.915f);
//ff rr motor rpm differ--

    if (fabsf(m_rpm1) < MOTOR_MIN_RPM) m_rpm1 = 0.0f;
    if (fabsf(m_rpm2) < MOTOR_MIN_RPM) m_rpm2 = 0.0f;

    if(m_rpm1>=MOTOR_MAX_RPM) m_rpm1=MOTOR_MAX_RPM;
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
    if(!disable_pid) {
        rpm = Dual_Motor_GetVelocty();
    } else {
        rpm.f1 = mLeftMotorPwm;
        rpm.f2 = mRigjtMotorPwm;
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

    // target_rpm 계산: CAN에서 받은 PWM을 그대로 사용 (PWM_UpdateRoutine이 이미 LogRamp 처리함)
    target_rpm.f1= (raw_target_pwm.f1 * MOTOR_MAX_RPM);
    target_rpm.f2= (raw_target_pwm.f2 * MOTOR_MAX_RPM);

    // 에러 계산 시 방향 일관성을 위해 current_rpm에 polar 보정 적용
    float current_rpm1_adj = (md_polarity.m1_dir==0) ? current_rpm1 : -current_rpm1;
    float current_rpm2_adj = (md_polarity.m2_dir==0) ? current_rpm2 : -current_rpm2;

    error_rpm.f1=(target_rpm.f1 - current_rpm1_adj);
    error_rpm.f2=(target_rpm.f2 - current_rpm2_adj);
    
    // 스로틀 정지 시 PID 완전 비활성화
    if (disable_pid || stop_throttle) {
        pid_pwm.f1 = 0.0f;
        pid_pwm.f2 = 0.0f;
    }
    else {
        pid_pwm.f1 = PID_Compute1(error_rpm.f1);
        pid_pwm.f2 = PID_Compute2(error_rpm.f2);
    }

    // PID 출력 폭주 방지: 최대 보정량 ±0.2f 제한 (2중 안전망)
    pid_pwm.f1 = fmaxf(fminf(pid_pwm.f1, 0.2f), -0.2f);
    pid_pwm.f2 = fmaxf(fminf(pid_pwm.f2, 0.2f), -0.2f);

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

    // EMB 체결 중이거나 EMB 해제 후 대기 중(0.2초)이면 PWM 차단 + 소프트스타트 초기화
#if 0
    if(brake_on_flag || emb_release_delay > 0) {
        set_pwm.f1 = set_pwm.f2 = 0;
        integral1 = integral2 = 0;
        emb_softstart_pwm = 0.0f;  // 해제 직후 0부터 시작
    }
#endif
    if (emb_softstart_pwm < 1.0f) {
        // 소프트스타트: PWM 크기 상한을 0→1.0으로 서서히 증가
        emb_softstart_pwm += EMB_SOFTSTART_RATE;
        if (emb_softstart_pwm > 1.0f) emb_softstart_pwm = 1.0f;
        set_pwm.f1 = copysignf(fminf(fabsf(set_pwm.f1), emb_softstart_pwm), set_pwm.f1);
        set_pwm.f2 = copysignf(fminf(fabsf(set_pwm.f2), emb_softstart_pwm), set_pwm.f2);
    }

    //printf("POLARITY: m1_dir[%d] m2_dir[%d] jenhujin[%d]\r\n",  md_polarity.m1_dir, md_polarity.m2_dir, pDataClass->ctl_pwm.io.toggle.jenhujin);
    // 듀얼 모터 PWM 출력 적용
    disable_pid=1;
    Dual_Motor_set_pwm10(md_polarity, set_pwm, disable_pid);

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
		if(fabsf(avr_rpm.f1)<400.0f) {
			if(m1_over_cnt)m1_over_cnt--;
		}
		else m1_over_cnt=10;

    	if(m1_over_cnt==0) m1_stop_rpm = 1;
   }
   
    if(!m2_stop_rpm && m2_stop_pwm){
		if(fabsf(avr_rpm.f2)<400.0f) {
			if(m2_over_cnt)m2_over_cnt--;
		}
		else m2_over_cnt=10;

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
    	m1_over_cnt = 10;
    }
    
    if (fabsf(stop_pwm2) > STOP_INPUT_PWM) {
    	m2_stop_pwm = 0;
    	m2_stop_rpm = 0;
    	m2_over_cnt = 10;
    }
    
    // 양쪽 중 하나라도 재가동되면 전체 브레이크 상태 초기화
    if (fabsf(stop_pwm1) > STOP_INPUT_PWM || fabsf(stop_pwm2) > STOP_INPUT_PWM) {
    	brake_delay_timeout=brake_delay_timeout_init;
    	integral1=integral2=0;
    	// EMB가 체결 상태였다면 해제 후 0.5초 대기 타이머 시작
    	if (brake_on_flag) {
    		emb_release_delay = EMB_RELEASE_DELAY_TICK;
    	}
    	brake_on_flag = 0;
    }

    // EMB 해제 대기 타이머 카운트다운 (10ms마다 호출)
    if (emb_release_delay > 0) emb_release_delay--;
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
