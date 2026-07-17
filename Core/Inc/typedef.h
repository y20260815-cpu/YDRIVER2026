#ifndef TYPEDEF_H
#define TYPEDEF_H

#include "stm32f1xx_hal.h"
#include <vector>
#include <ctime>
#include <string>
#include <cstdarg>

using namespace std;

#define DRIVER_BD2
#define ADC_SENSITIVITY 0.1f
#define ZIGBEE_REMAIN 40 //400ms
#define ZIGBEE_MY_ID 0xCC
#define MOTOR_MAX_RPM 2100.0f //3600.0f
#define MOTOR_MIN_RPM 30.0f

#define OPAMP_GAIN 0.0567f////A= R2/R3=6.8K/120k=0.0567
//#define OPAMP_GAIN 0.056f//adjust
#define DEADZONE_THRESHOLD 0.05f
#define STOP_INPUT_PWM 0.1f

enum ENUM_KIND_SWITCH{
	SW_JENHUJIN=0,
	SW_STOP,
	SW_LIFT,
	SW_BISANG,
};
enum ENUM_FM2000_KIND_SWITCH{
	FM2000_FNR1=0,
	FM2000_FNR2,
	FM2000_BISANG,
};

enum ENUM_DIR_SWITCH_STATE {
	CENTER=0,
	JENJIN,
	HUJIN,
};

enum ENUM_KEEP_STATE {
	KEEP_NONE=0,
	SPIN_LEFT,
	SPIN_RIGHT,
};

enum ENUM_IMMEDIATE_EVENT {
	 IE_NONE=0,
	TURN_LEFT,
	TURN_RIGHT,
	LIFT_LEFT,
	LIFT_RIGHT,
};

enum ENUM_CHANGE_EVENT {
	EVT_SW1_CENTER=0,
	EVT_SW1_JENJIN,
	EVT_SW1_HUJIN,
	EVT_SW2_CENTER_SPIN,
	EVT_SW2_LEFT_SPIN,
	EVT_SW2_RIGHT_SPIN,
	EVT_SW2_CENTER_TURN,
	EVT_SW2_LEFT_TURN,
	EVT_SW2_RIGHT_TURN,
//	EVT_SPIN_LEFT,
//	EVT_SPIN_RIGHT,
//	EVT_LIFT_LEFT,
//	EVT_LIFT_RIGHT,
};
//-----------------------------
typedef struct {
	int pwm_value;
	uint8_t  pwm_dir;
	uint8_t  pwm_run;
	uint8_t  pwm_en;
}PWM_PWM;

typedef struct {
	PWM_PWM pwm1;
	PWM_PWM pwm2;
}PWM_DATA;

typedef struct {
    uint8_t TOGGLE_A:2;
    uint8_t TOGGLE_B:2;
    uint8_t TOGGLE_C:2;
    uint8_t TOGGLE_D:2;
}BIT_PORT_INPUT;


typedef union _PORT_INPUT
{
	struct {
		uint8_t JENHUJIN1:2;
		uint8_t stopBTN_lr:2;
		uint8_t liftBTN_ud:2;
	};
	uint8_t u8;
}PORT_INPUT;

typedef struct _RELAY
{
  uint8_t MC1:1;
  uint8_t MC2:1;
  uint8_t BRAKE:1;
  uint8_t UP:1;
  uint8_t DN:1;
  uint8_t FAN1:1;
  uint8_t FAN2:1;
}RELAY;


typedef struct _BIT_FLAG
{
  uint8_t run:1;
  uint8_t trot_zero1:1;
  uint8_t stop1:1;
  uint8_t stop2:1;
  uint8_t left_spin:1;
  uint8_t right_spin:1;
  uint8_t x6:1;
  uint8_t x7:1;
}BIT_FLAG;

typedef union _FLAG_STATE
{
	struct {
	  uint8_t run:1;
	  uint8_t trot_zero:1;
	  uint8_t stop1:1;
	  uint8_t stop2:1;
	  uint8_t left_spin:1;
	  uint8_t right_spin:1;
	  uint8_t x6:1;
	  uint8_t x7:1;
	};
	uint8_t u8;
}FLAG_STATE;

typedef struct {
  int16_t adc1; //2byte
  int16_t adc2; //2byte
  BIT_PORT_INPUT remote_group_sw1;//1byte
  BIT_PORT_INPUT remote_group_sw2;//1byte
  RELAY remote_relay;
  uint8_t x7;
}STRUCT_RX_DATA;

typedef struct {
  int16_t pwm1;
  int16_t pwm2;
  uint8_t fet_temp;
  uint8_t motor_temp;
  uint8_t flg_state;
  uint8_t iPort;
}STRUCT_TX_DATA;

//------------MAIN-----------
typedef struct {
  FLAG_STATE flg_state;
  RELAY relay;
}MAIN_MAIN;

typedef union _BIT_SYSTEM_FLAG {
    struct {
    	uint8_t canReady:1;
    	uint8_t savedConfigOK:1;
    	uint8_t bisang:1;
    	uint8_t cruise:1;
    	uint8_t leap:1;
    	uint8_t stop_throttle:1;
    	uint8_t stop_rpm:1;
    	uint8_t SaveEEPROM:1;
    };
    uint8_t u8;
}BIT_SYSTEM_FLAG;

typedef union _CAN_UP_FLAG_DATA {
  struct {
    uint8_t emb_error : 1;
    uint8_t can_ready : 1;
    uint8_t bisang : 1;
    uint8_t emb : 1;
    uint8_t obstacle : 1;
    uint8_t limitSwitch : 2;
    uint8_t spray : 1;
  };
  uint8_t u8;
}CAN_UP_FLAG_DATA;

typedef union _BIT_CAN_FLAG {
    struct {
    	uint8_t needTxsetupData:1;
    	uint8_t needRxConfigData:1;
    	uint8_t x2:1;
    	uint8_t x3:1;
    	uint8_t x4:1;
    	uint8_t x5:1;
    	uint8_t x6:1;
    	uint8_t x7:1;
    };
    uint8_t u8;
}BIT_CAN_FLAG;

typedef struct _MY_BATTERY
{
	float use_battery_voltage;
	float measure_battery_voltage;
	float measure_emb_resister;
	float measure_m1_current;
	float measure_m2_current;
	float measure_m12_current;

	float motor_spec_voltage;
	float motor_spec_currente;
	float motor_protect_temperature;
	float motor_protect_current;
	int8_t fet_temp;
	int8_t motor_temp;
	int8_t protect_fet_temp;
	struct
	{
		uint8_t m1_error :1;
		uint8_t m2_error :2;
		uint8_t m1_high_temp :1;
		uint8_t m2_high_temp :1;
	}moter_error;
}MY_BATTERY;


typedef union _MY_TOGGLE {
  struct {
    uint8_t jenhujin : 2;
    uint8_t t_sw : 2;//stop_lr
    uint8_t toggle_3 : 2;
    uint8_t spray1 : 1;
    uint8_t spray2 : 1;
  };
  uint8_t u8;
} MY_TOGGLE;

typedef union _MY_BUTTON {
  struct {
    uint8_t ioMsg : 2;
    uint8_t lift_Xud: 2;
    uint8_t castUPDN: 2;
    uint8_t disablePID : 1;
    uint8_t emergency : 1;
  };
  uint8_t u8;
} MY_BUTTON;

typedef union _FM2000_BUTTON {
  struct {
    uint8_t FNR1: 2;
    uint8_t FNR2: 2;
    uint8_t x1:   2;
    uint8_t x2 :  1;
    uint8_t emergency : 1;
  };
  uint8_t u8;
} FM2000_BUTTON;

typedef struct _MY_IO
{
    MY_TOGGLE toggle;
    MY_BUTTON btn;
}MY_IO;


typedef union _LIFT_BUTTON {
  struct {
    uint8_t liftUPDN: 2;
    uint8_t castUPDN: 2;
    uint8_t xliftLimit: 2;
    uint8_t castLimit: 2;
  };
  uint8_t u8;
} LIFT_BUTTON;

//buf[0] = HI_UINT16(_LeftMotor_velocity);
//buf[1] = LO_UINT16(_LeftMotor_velocity);
//buf[2] = HI_UINT16(_limit_velocity);
//buf[3] = LO_UINT16(_limit_velocity);
//buf[4] = HI_UINT16(_RightMotor_velocity);
//buf[5] = LO_UINT16(_RightMotor_velocity);
//buf[6] = _my_toggle.u8;
//buf[7] = _my_button.u8;

typedef struct
{
	int16_t LeftMotor_velocity;
	int16_t limit_velocty;
    int16_t RightMotor_velocity;
    uint16_t canBrakeDelay;
    uint8_t  canBattery;
//    int16_t xx1;
    MY_TOGGLE toggle;
    MY_BUTTON btn;
}CAN_SERVICE_DATA_UNIT;

//-------------------------------
typedef struct _XY_POSITION
{
	int8_t x;
	int8_t y;
}XY_POSITION;

typedef struct _MOTOR_DIRECTION
{
	uint8_t m1_dir;
	uint8_t m2_dir;
}MOTOR_DIRECTION;

typedef struct _I_PWM
{
	int pwm1;
	int pwm2;
}I_PWM;

typedef struct _F_PWM
{
	float f1;
	float f2;
}DoubleF_VALUE;

typedef struct _POSITION_ANGLE
{
	DoubleF_VALUE source_pwm;
	DoubleF_VALUE target_pwm;
	DoubleF_VALUE limit_ratio;
	DoubleF_VALUE exSet_pwm;
	MOTOR_DIRECTION mdir;
	DoubleF_VALUE rpm;
	MY_IO io;
}POSITION_ANGLE;

//typedef struct {
//	uint8_t o3_error:1;
//	uint8_t door:1;
//	uint8_t sht20_error:1;
//	uint8_t t2_error:2;//0:normal 1: open 2:short 3:xx
//	uint8_t t3_error:2;//0:normal 1: open 2:short 3:xx
//	uint8_t exist:1;
//}DATA_BIT;
//
//typedef struct
//{
//  uint16_t bd_id;
//  int16_t o3_ppb;
//  int16_t temperature;
//  int16_t humidity;
//  int16_t t1;
//  int16_t t2;
//  int16_t x1;
//  int16_t x2;
//  DATA_BIT state_bit;
//}SENSOR_DATA;

//typedef struct
//{
//  uint8_t stx; // 1
//  uint8_t size;	//1
//  SENSOR_DATA data;
//  uint8_t sum; //1
//  uint8_t etx; //1
//}SENSOR_BOX_RS485_RESPONSE;


// I2C acknowledge
typedef enum{
  ACK      = 0,
  NO_ACK   = 1,
}etI2cAck;

typedef enum{
  LOW      = 0,
  HIGH     = 1,
}etI2cLevel;

// Error codes
typedef enum{
  ACK_ERROR                = 0x01,
  TIME_OUT_ERROR           = 0x02,
  CHECKSUM_ERROR           = 0x04,
  UNIT_ERROR               = 0x08
}etError;

typedef struct
{
  int8_t hours;
  int8_t minutes;
  int8_t seconds;
} I2C_BM8563_TimeTypeDef;


typedef enum
{
    NO_PRESS=0,
//	ADC_POWERX1_KEY,
//	ADC_MENUX1_KEY,
//	ADC_MENUX10_KEY,
//	ADC_LEFTX1_KEY,
//	ADC_LEFTX10_KEY,
//	ADC_LEFTX100_KEY,
//	ADC_RIGHTX1_KEY,
//	ADC_RIGHTX10_KEY,
//	ADC_RIGHTX100_KEY,
//	ADC_UPX1_KEY,
//	ADC_UPX10_KEY,
//	ADC_UPX100_KEY,
//	ADC_DNX1_KEY,
//	ADC_DNX10_KEY,
//	ADC_DNX100_KEY,
//	eJESANG_KEY,
//	MQ_JESANG_ON_KEY,
//	MQ_JESANG_OFF_KEY,
//	MQ_LCD_ON_KEY,
//	MQ_LCD_OFF_KEY,
//	KEY_ONLY_BEEP,
} eButtonEvent;



typedef struct
{
    uint8_t stx;
    uint8_t id;
    int16_t speed;
    int16_t limit_velocty;
    uint16_t tbd[3];
    MY_BUTTON btn1;
    MY_BUTTON btn2;
    uint8_t checksum;
    uint8_t etx;
}RCU_DATA_STRUCTURE;

typedef struct
{
    uint8_t stx;//1
    uint8_t id;//1
    int16_t leftMotor_velocity;//2
    int16_t limit_velocty;//2
    int16_t rightMotor_velocity;//2
    int16_t Vbat;//2
    int8_t ntc_fet;//1
    int8_t ntc_mot;//1
    int8_t dummy1; //1
    MY_BUTTON dbtn1;//1
    MY_BUTTON dbtn2;//1
    uint8_t rssi;//1
    uint8_t checksum;//1
    uint8_t etx;//1
}DCU_DATA_STRUCTURE;


typedef struct
{
	uint16_t motor_max_rpm;
	float rpm_to_pwm_scale;
	float Kp;
	float Ki;
	float Kd;
}PID_CONFIG;

typedef struct
{
	uint16_t idx;
	uint16_t battery_voltage;
	uint16_t limit_current;
	uint16_t limit_motor_temp;
	uint16_t limit_fet_temp;
	uint16_t alarm_Battery;
	uint16_t cart_type;
	uint16_t motor1_polarity;
	uint16_t motor2_polarity;
	uint16_t tbd;
	uint16_t reserved;  // conf2.idx padding (설정기 호환용)
	uint16_t tottle_offset;
	uint16_t stop_slip;
	uint16_t foreward;
	uint16_t backward;
	uint16_t accel;
	uint16_t decel;
	uint16_t brake_delay;
	uint16_t brake_rate;
	uint16_t checkSum;
}SYSTEM_CONF;


const SYSTEM_CONF sysConf_init =
{
	0,	//idx
	1,	//BATTERY VOLTAGE 0:12V 1:24V 2:36V 3:48V
	200,//Limit_CURRENT
	90,	//Limit MOTOR Temp
	85,	//Limit FET Temp
	23,	//Low BATTERY
	1,	//Wheel type
	0,	//MOTOR1 극성
	0,	//MOTOR2 극성
	99,	//tbd
	0,	//reserved (conf2.idx padding)
	300,//Trottle Offset
	80,	//stop_slip
	100,//forward
	80,	//backward
	10,	//Accel
	20,	//Decel
	400,//Brake Delay(ms)
	5,	//Brake Rate
	99	//checkSum
};

/////////////////////
typedef struct __attribute__((packed)) {
    uint16_t conf_id;
    uint16_t conf_wheel;
    uint16_t conf_vbat;
    uint16_t conf_low_bat;
    uint16_t conf_M1_polarity;
    uint16_t conf_M2_polarity;
    uint16_t conf_trottle_offset;
    uint16_t conf_trottle_inflect;
    uint16_t conf_fw_percent;
    uint16_t conf_rw_percent;
    uint16_t conf_accel;
    uint16_t conf_decel;
    uint16_t conf_current_limit;
    uint16_t conf_break_rate;
    uint16_t conf_break_delay;
    uint16_t conf_motor_temp;
    uint16_t conf_fet_temp;
  } CONF_CHUNK;

  typedef struct __attribute__((packed)) {
    uint8_t stx;
    uint8_t cmd;
    CONF_CHUNK chunk;
    uint8_t check_sum;
    uint8_t etx;
  } UPDATE_CHUNK;

////////////////////
  typedef enum {
      STATE_CAN=0,
      STATE_IO,
      STATE_WIFI,
  } SystemState;
#endif
