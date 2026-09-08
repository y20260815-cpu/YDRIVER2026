/*
 * dataclass.h
 *
 *  Created on: Jul 19, 2024
 *      Author: thpark
 */

#ifndef DATA_CLASS_DATACLASS_H_
#define DATA_CLASS_DATACLASS_H_
#include <stdint.h>

#define STABILIZE_DELAY_10MS_TICK 10
// 0.02 gave 0->100% in ~1 s (harsh launch); 0.012 stretches it to ~1.7 s.
#define ACCEL_RATE 0.012f
// Target-speed reduction per 10 ms, in CONTROL_TARGET_MAX_RPM units.
// 15.0/1500 = 1%/10ms -> full speed to 0 in about 1 s (same feel as the
// previous 7.2/720 tuning).
#define DECEL_RPM_PER_10MS 15.0f
// AS(0x701)/FM(0x702) byte2 controls acceleration time. Byte3 retains the
// battery-setting meaning used by the existing AS controller.
// master sends (300 / 24) map exactly to the tuned optimum above.
#define CAN_ACCEL_TIME_REF 300.0f
// CAN byte2 controls the full 0 -> 100% S-curve acceleration time.
// 10 = 1 s, 50 = 5 s, 200 = 20 s; values outside this range saturate.
#define CAN_ACCEL_TIME_MIN_RAW 10U
#define CAN_ACCEL_TIME_MAX_RAW 200U
#define CAN_ACCEL_TIME_DEFAULT_RAW 50U
// Existing AS ESP32 menu stores 50/100/150/200 and transmits menu / 5.
// The STM32 always calculates acceleration from the received raw byte.
#define CAN_ACCEL_LEGACY_MENU_SCALE 5U
// DC-Link rise limits relative to the voltage captured at stop entry.
#define DECEL_VDC_SLOW_RATIO 1.03f
#define DECEL_VDC_HOLD_RATIO 1.10f
// Keep a small controlled ramp at the voltage hold threshold so a neutral
// command cannot leave a permanent non-zero drive PWM.
#define DECEL_VDC_MIN_SCALE 0.10f
// Fixed delay before the electromagnetic brake is applied after PWM stops.
// Unit: milliseconds. Change this single value to retune the delay.
#define ELECTROMAGNETIC_BRAKE_DELAY_MS 300U
#define DEFAULT_PID_KP 0.8f

#define MAX_CURRENT 80.0f //MAX CURRENT //2.0f test
#define LIMIT_FET_TEMP 85
#define LIMIT_MOTOR_TEMP 85
#define FET_TEMP_DERATE_START 85
#define FET_TEMP_PWM_OFF 100
#define FET_TEMP_RESTART 80
#define OVER_CURRENT_RETRY_MAX 3
#define OVER_CURRENT_RETRY_WAIT_10MS_TICK 50
#define OVER_CURRENT_DETECT_10MS_TICK 1
#define MOTOR_COMMAND_QUEUE_DEPTH 8


enum ERROR_DISPLAY_CODE
{
	ERROR_CODE_0_NORMAL = 0,
	ERROR_CODE_1_LOW_VOLTAGE = 1,
	ERROR_CODE_2_OVER_VOLTAGE = 2,
	ERROR_CODE_3_FET_OVER_TEMPERATURE = 3,
	ERROR_CODE_4_MOTOR_OVER_TEMPERATURE = 4,
	ERROR_CODE_5_MOTOR1_FAULT = 5,
	ERROR_CODE_6_MOTOR2_FAULT = 6,
	ERROR_CODE_7_SHUNT_OVER_CURRENT = 7,
	ERROR_CODE_8_CAN_TIMEOUT = 8,
	ERROR_CODE_9_OVER_CURRENT = 9
};

typedef struct _ERROR_CODE_STATE
{
	uint8_t code;
	uint8_t normal;
	uint8_t low_voltage;
	uint8_t over_voltage;
	uint8_t fet_over_temperature;
	uint8_t motor_over_temperature;
	uint8_t motor1_fault;
	uint8_t motor2_fault;
	uint8_t tbd1;
	uint8_t can_timeout;
	uint8_t over_current;
} ERROR_CODE_STATE;

class data_class
{
private:
	typedef struct {
		float pwm1;
		float pwm2;
		uint32_t sequence;
	} MOTOR_COMMAND;

	MOTOR_COMMAND motor_command_queue[MOTOR_COMMAND_QUEUE_DEPTH];
	uint8_t motor_command_head = 0;
	uint8_t motor_command_tail = 0;
	uint8_t motor_command_count = 0;
	uint32_t motor_command_sequence = 0;
	float active_command_pwm1 = 0.0f;
	float active_command_pwm2 = 0.0f;
	float last_received_command_pwm1 = 0.0f;
	float last_received_command_pwm2 = 0.0f;
	uint8_t last_received_command_valid = 0;
	uint8_t motor_command_10ms_count = 0;
	float stop_vdc_reference = 0.0f;
	// Bit 0: M1, bit 1: M2.  A forward/reverse -> stop command keeps
	// subsequent queued targets blocked until the commanded PWM reaches zero.
	uint8_t stop_ramp_active_mask = 0;
	uint32_t accel_last_update_ms1 = 0;
	uint32_t accel_last_update_ms2 = 0;
	typedef struct {
		float start_pwm;
		float target_pwm;
		uint32_t elapsed_ms;
		uint8_t active;
	} S_CURVE_STATE;
	S_CURVE_STATE accel_curve1 = {0.0f, 0.0f, 0U, 0U};
	S_CURVE_STATE accel_curve2 = {0.0f, 0.0f, 0U, 0U};

	const float gamsok_ratio[12]={
		-1.0f,//100%역방향
		-0.8f,//1 :100%:080-->한쪽 정지
		-0.6f,//2 :100%:060-->한쪽 정지
		-0.4f,//3 :100%:040-->한쪽 정지
		-0.2f,//4 :100%:020-->한쪽 정지
		0.0f, //5 :100%:000-->한쪽 정지
		0.2f, //6 :020%:100--> 20%감속
		0.4f, //7 :040%:100--> 20%감속
		0.6f, //8 :060%:100--> 20%감속
		0.8f, //9 :080%:100--> 20%감속
		1.0f, //10:100%:100--> 20%감속
		1.0f  //11:1000%:100--> 20%감속
	};

	SystemState prev_state = STATE_IO;
	SystemState current_state = STATE_IO;
	uint8_t state_change_flag_timeout = STABILIZE_DELAY_10MS_TICK;
	LIFT_BUTTON local_lift;
	void stabilize_change();
	//void calcuration_stop_velocity();

	int16_t FormSensorMnt_CalCPUTemp(uint16_t value);
	int8_t get_ntc_temperature(uint16_t adc);
	void read_in_port();

	void relay_control();
	void lift_control(uint8_t Xlift, uint8_t Wlift);
	float get_voltage(uint16_t value);
	uint16_t get_m0_filter(uint16_t adc);
	uint16_t get_m1_filter(uint16_t adc);
	uint16_t get_m2_filter(uint16_t adc, float senstivity);
	uint16_t get_m3_filter(uint16_t adc, float senstivity);
	uint16_t get_m6_filter(uint16_t adc, float senstivity);//current
	uint16_t get_m7_filter(uint16_t adc, float senstivity);//dc-link voltage
	uint16_t get_m8_filter(uint16_t adc, float senstivity);
	uint16_t get_m9_filter(uint16_t adc, float senstivity);

	float get_wcs1600_current(uint16_t adc);
	void calibrate_wcs1600_zero();
	void update_current();
	void update_short_brake_1ms();
	void force_pwm_off_for_over_current();
	void update_error_code_once_per_second();
	void display_error_code_once_per_second();
	uint8_t fnd_can_dot_state = 0;
	void CAN_DCU_Information();
	void send_uart3_csv();
	//uint8_t get_state();
	//void change_function(uint8_t num);
	//void set_direction(uint8_t motor, uint8_t dir);
	//void change_subroutine(uint8_t state);
	float LogRampStep(float target, float current, uint32_t elapsed_ms);
	float PWM_UpdateRoutine(float targetPWM, float currentPWM, uint8_t throt_zero, uint32_t &last_update_ms, S_CURVE_STATE &curve);
	float get_fet_temp_pwm_scale();
	float scale_value(int value, int min_val, int max_val);
	float adc_to_pwm(uint16_t value);
	float Calcu_motor_command_pwm(float requested_pwm, uint8_t &confirmed_dir, float &current_pwm, uint8_t &dir_change_wait_cnt, uint32_t &last_update_ms, S_CURVE_STATE &curve);
	void ProcessMotorCommandQueue();
	void UpdateMotorCommandOutput();
	void ClearMotorCommandQueue();
	//void Get_IO_PARM();
	POSITION_ANGLE Detect_Change_evt(uint8_t toggle_jenhujin, uint8_t botton_LR);
	void ON_Board_INPUT();
	float Calcu_limit(float throttle, float limit);
//	DoubleF_VALUE Calcu_sourcePWM(float i_poten, float limit, uint8_t jenhujin, uint8_t stop_lr);
//	DoubleF_VALUE Calcu_targetPWM(float throttle, float limit, uint8_t jenhujin_key, uint8_t jenhujin_confirmed,
//	                              uint8_t stop_lr, uint8_t m1_pol, uint8_t m2_pol,
//	                              float fwd_ratio, float bwd_ratio);
	void Lift_Cast_Stop(void);
	void Lift_UpDn(uint8_t cmd);
	void Cast_UpDn(uint8_t cmd);
	LIFT_BUTTON Get_localGPIO();
	void pack7(uint8_t out[7], int16_t rpm1, int16_t rpm2, uint8_t current, uint16_t batt10, uint8_t motor_temp7, uint8_t fet_temp7);
	void unpack7(const uint8_t in[7], int16_t &rpm1, int16_t &rpm2, uint8_t &current, uint16_t &batt10, uint8_t &motor_temp7, uint8_t &fet_temp7);
	float get_emb_resister(uint16_t acc_adc, uint16_t emb_adc);
	void get_fm2000_port();
public:
	//CONF_CHUNK config_table;
	//uint16_t brake_delay_timeout=10;
	//uint8_t brake_delay_flag=1;
	uint16_t brake_delay;
	//uint16_t gamsok_idx;//=setup_data.conf2.brake_rate;
	// CAN으로 조정되는 램프 계수 (로컬 모드는 기본값 유지)
	float can_accel_rate = ACCEL_RATE;
	uint16_t can_accel_duration_10ms = CAN_ACCEL_TIME_DEFAULT_RAW * 10U;
	float can_decel_rpm_per_10ms = DECEL_RPM_PER_10MS;

	uint8_t motor1_polarity;
	uint8_t motor2_polarity;

	MAIN_MAIN gMAIN;
	BIT_SYSTEM_FLAG sysFlag;
	CAN_SERVICE_DATA_UNIT vcu_sdu;
	uint8_t can_byte2_raw = 0;
	uint8_t can_byte3_raw = 0;
	uint8_t pSpary=0;
	int16_t Potentio_val;
	int run_cnt=5;
	//uint8_t turn_stop_percent=0;
	uint8_t state=0;
	uint8_t ex_state=0;
	uint8_t PWR_ON_Flg=0;
	uint8_t HOLD_Emergency=0;
	uint8_t current_run_state=0;
	uint8_t keep_state=KEEP_NONE;
	uint8_t immediate_state=IE_NONE;
	uint8_t JENHUJIN_SWITCH_CHANGE_FLAG=0;
	ENUM_CHANGE_EVENT direction_evt=EVT_SW1_CENTER;
	int16_t trottle, limit_val;//, exStopPwm1=0, exStopPwm2=0;
	uint16_t ladcValue[16];
	float wcs1600_zero_adc = 1938.0f;

	DCU_DATA_STRUCTURE dcu_data;
	uint8_t system_start_flag=0;
	uint8_t ADC_flag=0;
	POSITION_ANGLE inputRaw;

	MY_IO board_io;
	MY_IO ex_board_io;
	MY_BATTERY batt;
	ERROR_CODE_STATE error_code;
	FM2000_BUTTON fm2000_gpio;
	float currentPWM1 = 0.0f;  // 전역 또는 클래스 멤버
	float currentPWM2 = 0.0f;  // 전역 또는 클래스 멤버
	uint8_t fm2000_dir1 = 0;
	uint8_t fm2000_dir2 = 0;
	uint8_t fm2000_dir_change_wait1 = 0;
	uint8_t fm2000_dir_change_wait2 = 0;
	uint8_t fet_temp_over_cnt = 0;  // FET 온도 초과 지속 카운터 (1초 단위)
	uint8_t fet_temp_protect = 0;   // FET 과열 보호 플래그 (1초 이상 초과시 power off)
	uint16_t over_current_10ms_cnt = 0;
	uint16_t over_current_retry_wait_10ms_cnt = 0;
	uint8_t over_current_retry_count = 0;
	uint8_t over_current_protect = 0;
	//float set_foreward;//=setup_data.conf2.foreward;
	//float set_backward;//set_backward_setup_data.conf2.backward;

	data_class();
	virtual ~data_class();
	void power_on();
	void power_off();
	void relayAllOFF();
	void one_millisec_routine();
	
	void ten_millisec_routine();
	void can_process_routine();
	void hnd_millisec_routine();
	void onesec_routine();
	//void change_direction(uint8_t spin);
	void Get_AdcData();
	void QueueMotorCommand(float pwm1, float pwm2);
	float GetStopVdcReference();
	uint8_t get_switch_port(uint8_t port);
	void MakeDCU_Data();
	int clamp(int value, int min, int max);
};

#endif /* DATA_CLASS_DATACLASS_H_ */
