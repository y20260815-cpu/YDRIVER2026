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

//#define MOTOR_VOLTAGE 24.0f
//#define MOTOR_MAX_RPM 3600.0f
#define VREF_MEASURE_TIME_OUT 10
#define FILTER_SENSITIVITY 0.1f
#define BEMF_SAMPLE_COUNT 1
#define BEMF_MEASURE_INTERVAL_TICK 8
#define BEMF_MEASURE_DUTY_MIN 0.12f
#define BEMF_ACTIVE_DUTY_MAX 0.65f
#define RPM_ESTIMATE_ACCEL_ALPHA 0.08f
#define RPM_ESTIMATE_DECEL_ALPHA 0.18f
#define RPM_MEASURE_BLEND_ALPHA 0.22f
#define RPM_ZERO_SNAP_THRESHOLD 18.0f
#define RPM_BIAS_BLEND_ALPHA 0.18f
#define RPM_BIAS_DECAY 0.94f
#define SPEED_HOLD_ENABLE_DUTY_MIN 0.18f
#define SPEED_HOLD_TRIM_ALPHA 0.35f
#define OVERSPEED_TRIM_GAIN 0.0008f
#define OVERSPEED_TRIM_MAX 0.35f
#define OVERSPEED_BRAKE_RPM 140.0f
#define OVERSPEED_COUNTER_TORQUE_GAIN 0.00035f
#define OVERSPEED_COUNTER_TORQUE_MAX 0.08f
#define SPEED_HOLD_CONFIDENCE_TICK 12
#define EMB_RELEASE_DELAY_MS  20   // EMB 해제 후 PWM 대기 시간 (ms)
#define EMB_RELEASE_DELAY_TICK (EMB_RELEASE_DELAY_MS / 10)  // 10ms 틱 기준
#define EMB_SOFTSTART_RATE    0.05f  // 매 10ms마다 5% 증가 → 0→100% = 200ms

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


//----------------------------
void DisableAllFETs_Dual();
	void Apply_PWM_Direct(DoubleF_VALUE fSetPwm);
float get_m1_filter(float value);
float get_m2_filter(float value);
DoubleF_VALUE Dual_Motor_GetVelocty();
uint8_t stop_throttle=0;
uint16_t brake_delay_timeout=100;
uint16_t brake_delay_timeout_init=100;
uint8_t stop_rpm_flag;
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
uint8_t bemf_measure_tick=0;

// EMB 해제 후 PWM 대기 타이머 (10ms 틱, 50 = 500ms)
uint16_t emb_release_delay=0;
// EMB 해제 후 소프트스타트 제한값 (0.0→1.0 점진 증가)
float emb_softstart_pwm=1.0f;

// Slew-rate: 1ms 루프에서 target을 천천히 추종하는 현재 출력값
DoubleF_VALUE slew_pwm;
DoubleF_VALUE rpm_model_bias;
DoubleF_VALUE rpm_measured;
DoubleF_VALUE speed_hold_trim_pwm;
uint8_t speed_hold_confidence=0;
uint8_t speed_hold_active1=0;
uint8_t speed_hold_active2=0;

    //float ki1, kd1;
    float integral1, prev_error1;
    //float ki2, kd2;
    float integral2, prev_error2;
    float PID_Compute1(float error);
    float PID_Compute2(float error);
    float Update_RPM_Channel(float current, float model, float bias);
    void Update_RPM_Bias(DoubleF_VALUE model_rpm, DoubleF_VALUE measured_rpm);
    void Update_RPM_Estimator(DoubleF_VALUE fSetPwm);
    float Apply_SpeedHold_Channel(float raw_pwm, float out_pwm, float target_rpm, float current_rpm, float *trim_pwm, uint8_t *active_flag);
    DoubleF_VALUE Apply_SpeedHold(DoubleF_VALUE raw_target_pwm, DoubleF_VALUE out_pwm, DoubleF_VALUE current_rpm_adj);
    DoubleF_VALUE pidCalibration(DoubleF_VALUE input, DoubleF_VALUE pid_pwm);
    void pwm8_out(int pwm1, int pwm2, int pwm3, int pwm4, int pwm5, int pwm6, int pwm7, int pwm8);

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
float vref1_measured=2.5f, vref2_measured=2.5f;
float bemf_offset1, bemf_offset2;
uint8_t vref_measure_timeout=VREF_MEASURE_TIME_OUT;
int decelElapsedTime=0;

virtual ~PWM16();
void PWM16_START();
void Update_PWM(uint8_t disable_pid, float target_pwm1, float target_pwm2);
void Dual_Motor_set_pwm10(MOTOR_DIRECTION dm_polar, DoubleF_VALUE fSetPwm);
//void Get_Motor_Vref(uint8_t time_out);
void CheckBrakeState(float stop_pwm1, float stop_pwm2);
void Calibrate_BEMF_Offset();
void PWM_1ms();
int ex_pwm1, ex_pwm2, ex_pwm3, ex_pwm4, ex_pwm5, ex_pwm6, ex_pwm7, ex_pwm8;
int set_pwm1, set_pwm2, set_pwm3, set_pwm4, set_pwm5, set_pwm6, set_pwm7, set_pwm8;
};

#endif /* PWM16_PWM16_H_ */
