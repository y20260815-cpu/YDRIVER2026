/*
 * PWM16.h
 *
 *  Created on: Dec 31, 2023
 *      Author: thpark
 */

#ifndef PWM16_PWM16_H_
#define PWM16_PWM16_H_

#define PWM_SET_TIME_OUT 4
// 1ms마다 target PWM을 추종하는 최대 변화량 (0.0~1.0 기준)
// ex) 0.1f -> 10ms 만에 0->full 도달 (10스텝 × 0.1 = 1.0)
#define SLEW_RATE_PER_MS  0.1f

#define FET_MAX 870//850
#define PWM_DAED_TIME 50//100 //dead time
#define PID_LOW_SPEED_THRESHOLD  500.0f  // RPM 이하에서만 PID 적용
#define VREF_MEASURE_TIME_OUT 10
#define FILTER_SENSITIVITY 0.1f
#define EMB_RELEASE_DELAY_MS  200   // EMB 해제 후 PWM 대기 시간 (ms)
#define EMB_RELEASE_DELAY_TICK (EMB_RELEASE_DELAY_MS / 10)  // 10ms 틱 기준
#define EMB_SOFTSTART_RATE    0.05f  // 매 10ms마다 5% 증가 → 0→100% = 200ms

// PWM short-brake control (1 ms update)
#define SHORT_BRAKE_RAMP_PER_MS       0.0003f
#define SHORT_BRAKE_INITIAL_DUTY       0.01f
#define SHORT_BRAKE_VDC_RISE_START    0.2f
#define SHORT_BRAKE_VDC_FULL_RATIO    0.08f
#define SHORT_BRAKE_VDC_MAX_RATIO     0.10f
#define SHORT_BRAKE_VDC_WARN          45.0f
#define SHORT_BRAKE_VDC_PROTECT       52.0f
#define SHORT_BRAKE_VDC_HARD          58.0f
#define SHORT_BRAKE_DUTY_VDC_STEP     0.01f
#define SHORT_BRAKE_DUTY_WARN_STEP    0.02f
#define SHORT_BRAKE_DUTY_FAST_STEP    0.05f
#define SHORT_BRAKE_DUTY_RELEASE_STEP 0.002f
#define SHORT_BRAKE_PWM_PERIOD        1024

class PWM16
{
private:
int decay_cnt=PWM_SET_TIME_OUT;
int m1_pwm,m2_pwm;
float m1_value=0.0f, m2_value=0.0f;  // 초기화 추가
//uint8_t motor_dir1, motor_dir2;
float fm1_pwm, fm2_pwm;
float f_exPWM1,f_exPWM2;

uint8_t prev_direction1 = 0xFF;
uint8_t prev_direction2 = 0xFF;
DoubleF_VALUE avr_rpm;//

DoubleF_VALUE fpwm;
DoubleF_VALUE fex_pwm;
I_PWM ipwm;
int tt_out=10;
DoubleF_VALUE Button_Stop_process(I_PWM iPWM, uint8_t stop_bt, uint8_t dir);
MOTOR_DIRECTION mot_dir;
//----------------------------
//MOTOR_DIRECTION pub_md_polarity;
//DoubleF_VALUE pub_set_pwm;

int pub_pwm1;
int pub_pwm2;
int pub_pwm3;
int pub_pwm4;
int pub_pwm5;
int pub_pwm6;
int pub_pwm7;
int pub_pwm8;
//----------------------------
void DisableAllFETs_Dual();
	void Apply_PWM_Direct(DoubleF_VALUE fSetPwm);
float get_m1_filter(float value);
float get_m2_filter(float value);
DoubleF_VALUE Dual_Motor_GetVelocty();
uint8_t stop_throttle=0;
uint16_t brake_delay_timeout=100;
uint16_t brake_delay_timeout_init=100;
uint8_t over_cnt=10;

// 각 모터별 브레이크 상태 추적
uint8_t m1_stop_pwm=0;    // M1 PWM 정지 감지
uint8_t m2_stop_pwm=0;    // M2 PWM 정지 감지
uint8_t m1_stop_rpm=0;    // M1 RPM 정지 감지
uint8_t m2_stop_rpm=0;    // M2 RPM 정지 감지
uint8_t m1_over_cnt=10;   // M1 RPM 안정화 카운터
uint8_t m2_over_cnt=10;   // M2 RPM 안정화 카운터

// BEMF 측정 중 플래그 (측정 중에는 Apply_PWM_Direct 차단)
uint8_t bemf_busy=0;

// EMB 해제 후 PWM 대기 타이머 (10ms 틱, 50 = 500ms)
uint16_t emb_release_delay=0;
// EMB 해제 후 소프트스타트 제한값 (0.0→1.0 점진 증가)
float emb_softstart_pwm=1.0f;

// Per-motor low-side PWM short-brake state
float short_brake_base_duty_m1=0.0f;
float short_brake_base_duty_m2=0.0f;
float short_brake_duty_m1=0.0f;
float short_brake_duty_m2=0.0f;
float short_brake_vdc_reference_m1=0.0f;
float short_brake_vdc_reference_m2=0.0f;
float short_brake_entry_pwm_m1=0.0f;
float short_brake_entry_pwm_m2=0.0f;
uint8_t short_brake_active_m1=0;
uint8_t short_brake_active_m2=0;

// DC-link voltage measurement is shared.
float short_brake_vdc_filter=0.0f;
uint8_t short_brake_filter_active=0;
void ApplyShortBrakeDuty(uint8_t brake_m1, float duty_m1,
		uint8_t brake_m2, float duty_m2);
float UpdateOneShortBrake_1ms(uint8_t brake_request,
		float dc_link_voltage, float target_pwm_abs,
		float &base_duty, float &output_duty,
		float &vdc_reference, float &entry_pwm, uint8_t &active);

// Slew-rate: 1ms 루프에서 target을 천천히 추종하는 현재 출력값
DoubleF_VALUE slew_pwm;

    //float ki1, kd1;
    float integral1, prev_error1;
    //float ki2, kd2;
    float integral2, prev_error2;
    float PID_Compute1(float error);
    float PID_Compute2(float error);
    DoubleF_VALUE pidCalibration(uint8_t disable_pid, DoubleF_VALUE input, DoubleF_VALUE pid_pwm);

public:
PWM16();
PWM_DATA pwm;
DoubleF_VALUE rpm;
DoubleF_VALUE target_rpm;
DoubleF_VALUE error_rpm;
DoubleF_VALUE pid_pwm;

// 10ms Update_PWM이 계산한 최종 목표 PWM (1ms slew가 추종할 target)
DoubleF_VALUE target_slew_pwm;

uint8_t brake_on_flag=0;
uint8_t stop_rpm_flag=0;  // BEMF RPM 감속 완료 플래그 (avr_rpm < 400 × 10샘플)
float vref1_measured=2.5f, vref2_measured=2.5f;
float bemf_offset1, bemf_offset2;
uint8_t vref_measure_timeout=VREF_MEASURE_TIME_OUT;
int decelElapsedTime=0;

virtual ~PWM16();
void PWM16_START();
void Update_PWM(uint8_t disable_pid, float target_pwm1, float target_pwm2);
void Dual_Motor_set_pwm10(MOTOR_DIRECTION dm_polar, DoubleF_VALUE fSetPwm, uint8_t disable_pid);
//void Get_Motor_Vref(uint8_t time_out);
void CheckBrakeState(float stop_pwm1, float stop_pwm2);
void UpdateShortBrakeControl_1ms(uint8_t brake_m1, uint8_t brake_m2,
		float target_pwm_abs_m1, float target_pwm_abs_m2,
		float dc_link_voltage);
void ResetShortBrakeControl();
void Calibrate_BEMF_Offset();
//void PWM_1ms();
};

#endif /* PWM16_PWM16_H_ */
