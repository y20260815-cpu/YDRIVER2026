/*
 * PWM16.cpp
 *
 *  Created on: Dec 31, 2023
 *      Author: thpark
 */
#include "extern.h"
#include "PWM16.h"
#include <math.h>

#define FET_MAX 850
#define PWM_DIRECT_CONTROL 100 //dead time

PWM16 *pPWM;

#define INTEGRAL_MAX_LIMIT 5000.0f  // 적분값 최대 제한

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
	// 슬루 레이트 초기값: 0으로 초기화하지 않으면 쓰레기값에서 슬루 시작
	ex_pwm1=ex_pwm2=ex_pwm3=ex_pwm4=0;
	ex_pwm5=ex_pwm6=ex_pwm7=ex_pwm8=0;
	set_pwm1=set_pwm2=set_pwm3=set_pwm4=0;
	set_pwm5=set_pwm6=set_pwm7=set_pwm8=0;
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
	htim2.Instance->CCR2=1024;
	htim2.Instance->CCR3=0;
	htim2.Instance->CCR4=1024;
	htim3.Instance->CCR1=0;
	htim3.Instance->CCR2=1024;
	htim3.Instance->CCR3=0;
	htim3.Instance->CCR4=1024;
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
    bemf_offset1 = 38.35f;  // (43.82 + 43.29) / 2
    bemf_offset2 = 38.72f;  // 동일하게 설정

    DisableAllFETs_Dual();
	delay_us(5);//	waiting for stable

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

	_rpm.f1=m_rpm1;
	_rpm.f2=m_rpm2;
    //cprintf(C_RED,"@@@@ vf1_measured: %.2f^%.2f rpm(%.0f|%.0f) vBEMF( %.2f| %.2f) vf_measured(%.2f|%.2f) bemf_offset(%.2f|%.2f)\r\n",vf1_measured, vf2_measured, _rpm.f1, _rpm.f2, vBEMF1, vBEMF2, vf1_measured, vf2_measured, bemf_offset1, bemf_offset2);
	return _rpm;
}
#if 0
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

#endif
void PWM16::Dual_Motor_set_pwm10(MOTOR_DIRECTION dm_polar, DoubleF_VALUE fSetPwm, uint8_t disable_pid)
{
	int mLeftMotorPwm, mRigjtMotorPwm, pwm1,pwm2,pwm3,pwm4,pwm5,pwm6,pwm7,pwm8;
	uint8_t dir1, dir2;
	//static uint16_t rpm_cnt=0;

	mLeftMotorPwm =  (int)(fminf(fabsf(fSetPwm.f1), 1.0f) * FET_MAX);
    mRigjtMotorPwm = (int)(fminf(fabsf(fSetPwm.f2), 1.0f) * FET_MAX);

   //mRigjtMotorPwm =mLeftMotorPwm; //for test
//-------------------------------------------------------------------
    //속도제한
    //mPwm1 = (int)(mLeftMotorPwm *  pDataClass->ctl_pwm.limit_ratio.fpwm1);
    //mPwm2 = (int)(mPwm2 *  pDataClass->ctl_pwm.limit_ratio.fpwm2);

//    switch(pDataClass->ctl_pwm.io.toggle.jenhujin)
//    {
//    case 0://정지
//    	break;
//    case 1://전진
//        mPwm1 = (int)(mPwm1 * pDataClass->set_foreward);
//        mPwm2 = (int)(mPwm2 * pDataClass->set_foreward);
//    	break;
//    case 2://후진
//        mPwm1 = (int)(mPwm1 * pDataClass->set_backward);
//        mPwm2 = (int)(mPwm2 * pDataClass->set_backward);
//    	break;
//    default:
//    	break;
//    }
//-----------------------------------------------------------------------
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
#if 1
//BEMF++
{
    static uint8_t  high_duty_mode = 0;  // 0=BEMF측정, 1=PWM추정
    static uint8_t  low_duty_cnt   = 0;  // 저duty 연속 횟수 (글리치 방지)
    static uint16_t rpm_cnt        = 0;  // 측정 주기 카운터

    float duty_max = fmaxf(fabsf(fSetPwm.f1), fabsf(fSetPwm.f2));

    // 고속 진입: duty > 0.65 즉시 전환 (1회 글리치 방지)
    if (!high_duty_mode && duty_max > 0.65f) {
        high_duty_mode = 1;
        low_duty_cnt   = 0;
    }
    // 저속 복귀: duty < 0.55 이 3회(30ms) 연속일 때만 전환 (순간 글리치 차단)
    if (high_duty_mode) {
        if (duty_max < 0.55f) {
            if (++low_duty_cnt >= 3) { high_duty_mode = 0; low_duty_cnt = 0; }
        } else {
            low_duty_cnt = 0;
        }
    }

   // if (!disable_pid) {
        rpm_cnt++;
        if (rpm_cnt > 2) {
            rpm_cnt = 0;
            float new_rpm1, new_rpm2;
            if (!high_duty_mode) {
                // 저속: BEMF 직접 측정
                DoubleF_VALUE m = Dual_Motor_GetVelocty();
                new_rpm1 = m.f1;
                new_rpm2 = m.f2;
            } else {
                // 고속: PWM으로 RPM 추정 (FET OFF 없음)
                new_rpm1 = fSetPwm.f1 * MOTOR_MAX_RPM;
                new_rpm2 = fSetPwm.f2 * MOTOR_MAX_RPM;
            }
            // EMA 필터: 급격한 전환 완화
            const float alpha = 0.3f;
            rpm.f1 = rpm.f1 * (1.0f - alpha) + new_rpm1 * alpha;
            rpm.f2 = rpm.f2 * (1.0f - alpha) + new_rpm2 * alpha;
        }
   // }
}
//BEMF--
#endif
//M1
	if(dir1==0){
		pwm1=pwm2=mLeftMotorPwm;
		pwm2=pwm1+PWM_DIRECT_CONTROL;
		pwm3=pwm4=0;
	}
	else {
		pwm3=pwm4=mLeftMotorPwm;
		pwm4=pwm3+PWM_DIRECT_CONTROL;
		pwm1=pwm2=0;
	}
//M2
	if(dir2==0){
		pwm5=pwm6=mRigjtMotorPwm;
		pwm6=pwm5+PWM_DIRECT_CONTROL;
		pwm7=pwm8=0;
	}
	else {
		pwm7=pwm8=mRigjtMotorPwm;
		pwm8=pwm7+PWM_DIRECT_CONTROL;
		pwm5=pwm6=0;
	}
	//if(pwm==0)pwm1=pwm2=pwm3=pwm4=0;
    set_pwm1 = pwm1;
    set_pwm2 = pwm2;    
    set_pwm3 = pwm3;
    set_pwm4 = pwm4;
    set_pwm5 = pwm5;
    set_pwm6 = pwm6;
    set_pwm7 = pwm7;
    set_pwm8 = pwm8;

    //pwm8_out(set_pwm1, set_pwm2, set_pwm3, set_pwm4, set_pwm5, set_pwm6, set_pwm7, set_pwm8);
	// htim3.Instance->CCR1=pwm1;
	// htim3.Instance->CCR2=pwm2;
	// htim3.Instance->CCR3=pwm3;
	// htim3.Instance->CCR4=pwm4;

	// htim2.Instance->CCR1=pwm5;
	// htim2.Instance->CCR2=pwm6;
	// htim2.Instance->CCR3=pwm7;
	// htim2.Instance->CCR4=pwm8;
}

void PWM16::pwm8_out(int pwm1, int pwm2, int pwm3, int pwm4, int pwm5, int pwm6, int pwm7, int pwm8)
{
    // Slew rate limiter: |diff| <= 10 → ±1/call, |diff| > 10 → ±5/call
    auto slew = [](int cur, int tgt) -> int {
        int diff = tgt - cur;
        if (diff == 0) return cur;
        int step = ((diff > 10) || (diff < -10)) ? 5 : 1;
        return cur + ((diff > 0) ? step : -step);
    };

    int r1 = slew(ex_pwm1, pwm1);
    int r2 = slew(ex_pwm2, pwm2);
    int r3 = slew(ex_pwm3, pwm3);
    int r4 = slew(ex_pwm4, pwm4);
    int r5 = slew(ex_pwm5, pwm5);
    int r6 = slew(ex_pwm6, pwm6);
    int r7 = slew(ex_pwm7, pwm7);
    int r8 = slew(ex_pwm8, pwm8);
#if 1
    // 10회 PWM 히스토리 기록 (슬루 적용 후 실제 출력값)
    static int hist[10][8] = {};
    static uint8_t idx = 0;

    hist[idx][0]=r1; hist[idx][1]=r2; hist[idx][2]=r3; hist[idx][3]=r4;
    hist[idx][4]=r5; hist[idx][5]=r6; hist[idx][6]=r7; hist[idx][7]=r8;

    if (idx == 9) {
        printf("--- PWM10 history (ch1~8) ---\r\n");
        for (int r = 0; r < 10; r++) {
            printf("[%2d] %4d %4d %4d %4d | %4d %4d %4d %4d\r\n",
                r, hist[r][0], hist[r][1], hist[r][2], hist[r][3],
                   hist[r][4], hist[r][5], hist[r][6], hist[r][7]);
        }
    }
    idx = (idx + 1) % 10;
#endif
    htim3.Instance->CCR1 = r1;
    htim3.Instance->CCR2 = r2;
    htim3.Instance->CCR3 = r3;
    htim3.Instance->CCR4 = r4;

    htim2.Instance->CCR1 = r5;
    htim2.Instance->CCR2 = r6;
    htim2.Instance->CCR3 = r7;
    htim2.Instance->CCR4 = r8;

    ex_pwm1 = r1;  ex_pwm2 = r2;  ex_pwm3 = r3;  ex_pwm4 = r4;
    ex_pwm5 = r5;  ex_pwm6 = r6;  ex_pwm7 = r7;  ex_pwm8 = r8;
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

    md_polarity.m1_dir = 0;
    md_polarity.m2_dir = 0;
	brake_delay_timeout_init=pDataClass->brake_delay/10;

	// NOTE: CheckBrakeState는 ten_millisec_routine()에서 Update_PWM 호출 후 별도 호출됨
	// 이곳에서 중복 호출 시 brake 타이머가 2배 속도로 감소하는 버그 발생 → 제거

	raw_target_pwm.f1 = target_pwm1;  // -1.0~1.0
	raw_target_pwm.f2 = target_pwm2;

	// 모터별 독립 PID 정지 판단 (한쪽만 0이어도 해당 모터 PID 동결)
	bool m1_zero_cmd = (fabsf(raw_target_pwm.f1) <= DEADZONE_THRESHOLD);
	bool m2_zero_cmd = (fabsf(raw_target_pwm.f2) <= DEADZONE_THRESHOLD);

	if (stop_throttle || m1_zero_cmd) integral1 = 0;
	if (stop_throttle || m2_zero_cmd) integral2 = 0;
	
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
    
    // 스로틀 정지 시 PID 완전 비활성화 - 모터별 독립 처리
    pid_pwm.f1 = (stop_throttle || m1_zero_cmd) ? 0.0f : PID_Compute1(error_rpm.f1);
    pid_pwm.f2 = (stop_throttle || m2_zero_cmd) ? 0.0f : PID_Compute2(error_rpm.f2);

#if 1
    // PID 출력 보정 적용 - pid_pwm은 이미 모터별로 0 처리되어 있음
    if(disable_pid || (stop_throttle && m1_zero_cmd && m2_zero_cmd)){
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
    if(brake_on_flag || emb_release_delay > 0) {
        set_pwm.f1 = set_pwm.f2 = 0;
        integral1 = integral2 = 0;
        emb_softstart_pwm = 0.0f;  // 해제 직후 0부터 시작
    }
    else if (emb_softstart_pwm < 1.0f) {
        // 소프트스타트: PWM 크기 상한을 0→1.0으로 서서히 증가
        emb_softstart_pwm += EMB_SOFTSTART_RATE;
        if (emb_softstart_pwm > 1.0f) emb_softstart_pwm = 1.0f;
        set_pwm.f1 = copysignf(fminf(fabsf(set_pwm.f1), emb_softstart_pwm), set_pwm.f1);
        set_pwm.f2 = copysignf(fminf(fabsf(set_pwm.f2), emb_softstart_pwm), set_pwm.f2);
    }

    //printf("POLARITY: m1_dir[%d] m2_dir[%d] jenhujin[%d]\r\n",  md_polarity.m1_dir, md_polarity.m2_dir, pDataClass->ctl_pwm.io.toggle.jenhujin);
    // 듀얼 모터 PWM 출력 적용
    Dual_Motor_set_pwm10(md_polarity, set_pwm, disable_pid);

#if 0
    printf("@TogStop[%d^%d] curRPM[%+05d^%+05d] targetPWM[%+03.2f^%+03.2f] targetRPM[%+05d^%+05d] errRPM[%+05d^%+05d] pidPWM[%+03.2f^%+03.2f] setPWM[%+03.2f^%+03.2f] BK_delay[%02d] stopRPM[%d] stopTHRO[%d]  ovCNT[%02d] avrRPM[%+05d^%+05d] bakF[%d] disable_pid[%d]\r\n",
			pDataClass->inputRaw.io.toggle, pDataClass->inputRaw.io.toggle.t_sw,
			(int)rpm.f1, (int)rpm.f2,raw_target_pwm.f1, raw_target_pwm.f2,
			(int)target_rpm.f1, (int)target_rpm.f2,
			(int)error_rpm.f1, (int)error_rpm.f2,
			pid_pwm.f1, pid_pwm.f2,	set_pwm.f1, set_pwm.f2,
			brake_delay_timeout, stop_rpm_flag, stop_throttle, over_cnt,(int)avr_rpm.f1, (int)avr_rpm.f2, brake_on_flag, disable_pid);
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

void PWM16::PWM_1ms(){
   pwm8_out(set_pwm1, set_pwm2, set_pwm3, set_pwm4, set_pwm5, set_pwm6, set_pwm7, set_pwm8);

}
