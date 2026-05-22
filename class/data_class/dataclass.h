/*
 * dataclass.h
 *
 *  Created on: Jul 19, 2024
 *      Author: thpark
 */

#ifndef DATA_CLASS_DATACLASS_H_
#define DATA_CLASS_DATACLASS_H_
#define STABILIZE_DELAY_10MS_TICK 10

class data_class
{
private:
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
	uint16_t get_voltage(uint16_t value);
	uint16_t get_m0_filter(uint16_t adc);
	uint16_t get_m1_filter(uint16_t adc);
	uint16_t get_m2_filter(uint16_t adc);
	void CAN_DCU_Information();
	//uint8_t get_state();
	//void change_function(uint8_t num);
	//void set_direction(uint8_t motor, uint8_t dir);
	//void change_subroutine(uint8_t state);
	float LogRampStep(float target, float current);
	float PWM_UpdateRoutine(float targetPWM, float currentPWM, uint8_t throt_zero);
	float scale_value(int value, int min_val, int max_val);
	//void Get_IO_PARM();
	POSITION_ANGLE Detect_Change_evt(uint8_t toggle_jenhujin, uint8_t botton_LR);
	void ON_Board_INPUT();
	POSITION_ANGLE CAN_Board_INPUT(bool isCan);
	float Calcu_limit(float throttle, float limit);
	DoubleF_VALUE Calcu_sourcePWM(float i_poten, float limit, uint8_t jenhujin, uint8_t stop_lr);
	void Lift_Cast_Stop(void);
	void Lift_UpDn(uint8_t cmd);
	void Cast_UpDn(uint8_t cmd);
	LIFT_BUTTON Get_localGPIO();
	void pack7(uint8_t out[7], int16_t rpm1, int16_t rpm2, uint8_t current, uint16_t batt10, uint8_t motor_temp7, uint8_t fet_temp7);
	void unpack7(const uint8_t in[7], int16_t &rpm1, int16_t &rpm2, uint8_t &current, uint16_t &batt10, uint8_t &motor_temp7, uint8_t &fet_temp7);
	float get_emb_resister(uint16_t acc_adc, uint16_t emb_adc);
public:
	//CART_SETUP cart_config;
	CONFIG_TOTAL setup_data;
	CONF_CHUNK config_table;
	float accel_rate, decel_rate;
	//uint16_t brake_delay_timeout=10;
	//uint8_t brake_delay_flag=1;
	uint16_t brake_delay;
	uint16_t gamsok_idx;//=setup_data.conf2.brake_rate;

	uint8_t motor1_polarity;
	uint8_t motor2_polarity;

	MAIN_MAIN gMAIN;
	BIT_SYSTEM_FLAG sysFlag;
	CAN_SERVICE_DATA_UNIT vcu_sdu;
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

	DCU_DATA_STRUCTURE dcu_data;
	uint8_t system_start_flag=0;
	uint8_t ADC_flag=0;
	POSITION_ANGLE inputRaw;

	MY_IO board_io;
	MY_IO ex_board_io;
	MY_BATTERY batt;

	float currentPWM1 = 0.0f;  // 전역 또는 클래스 멤버
	float currentPWM2 = 0.0f;  // 전역 또는 클래스 멤버
	float set_foreward;//=setup_data.conf2.foreward;
	float set_backward;//set_backward_setup_data.conf2.backward;

	data_class();
	virtual ~data_class();
	void power_on();
	void power_off();
	void relayAllOFF();
	void one_millisec_routine();
	void can_process_routine();
	
	void ten_millisec_routine();
	void hnd_millisec_routine();
	void onesec_routine();
	//void change_direction(uint8_t spin);
	void Get_AdcData();
	uint8_t get_switch_port(uint8_t port);
	void MakeDCU_Data();
	int clamp(int value, int min, int max);
};

#endif /* DATA_CLASS_DATACLASS_H_ */
