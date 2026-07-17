/*
 * dataclass.cpp
 *
 *  Created on: Jul 19, 2024
 *      Author: thpark
 */
#include "extern.h"
#include "dataclass.h"
#include "math.h"

//#define B_OFFSET 300 //UMJI_Thottle

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

	gMAIN.flg_state.trot_zero=1;
	relayAllOFF();
}

data_class::~data_class()
{
	// TODO Auto-generated destructor stub
}

void data_class::power_on(){
	PWR_ON_Flg=1;
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
	gMAIN.relay.MC2=1;
	HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	gMAIN.relay.MC1=1;
	HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_SET);
	pPWM->brake_on_flag=0;

	//motor1_polarity=sysConf.motor1_polarity;
	//motor2_polarity=sysConf.motor2_polarity;

	//gamsok_idx=sysConf.brake_rate;
	brake_delay=DEFAULT_BRAKE_DELAY;
	//set_foreward=sysConf.foreward/100.0f;
	//set_backward=set_foreward;
	batt.use_battery_voltage=DEFAULT_BATTERY_VOLTAGE;
	batt.motor_spec_voltage=DEFAULT_BATTERY_VOLTAGE;
	//batt.motor_spec_rpm=MOTOR_MAX_RPM;
	pidCONF.rpm_to_pwm_scale=1.0f/MOTOR_MAX_RPM;
	pidCONF.Kp=DEFAULT_PID_KP;


	//pPWM->pid_k
    //sysConf.motor1_polarity = 1;  // 0 → 1로 변경
    //sysConf.motor2_polarity = 1;  // 0 → 1로 변경
}

void data_class::power_off(){
	PWR_ON_Flg=0;
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_RESET);//FET ALL OFF
	gMAIN.relay.MC1=0;
	HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_RESET);//Main Relay Off
	gMAIN.relay.MC2=0;
	HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_RESET);//Aux Relay Off
	pPWM->brake_on_flag=0;
	HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);//EMB No Power
	HAL_Delay(500);
	relayAllOFF();
}

void data_class::relayAllOFF()
{
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
	if(gMAIN.relay.MC1) HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_RESET);

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


float data_class::LogRampStep(float target, float current)
{
    float error = target - current;
    if (fabsf(error) < 0.01f)   return 0.0f;
    // 감속 판단: 목표 절대값이 현재 절대값보다 작으면 감속 (부호 무관)
    float k = (fabsf(target) < fabsf(current)) ? DECEL_RATE : ACCEL_RATE;
    // 스텝 계산: 오차가 클 때는 큰 스텝, 작을 때는 작은 스텝
    float step = k * logf(fabsf(error) + 1.0f);
    // 오차 부호에 맞게 스텝 부호 적용
    if (error < 0)
        step = -step;
    return step;
}

float data_class::PWM_UpdateRoutine(float targetPWM, float currentPWM, uint8_t throt_zero)
{
    // 제동 모드 활성 시 목표 PWM을 0으로 설정
    //if (brakeFlag)
    if(throt_zero)   targetPWM = 0.0f;
    // 로그 기반으로 변화량(스텝) 계산
    float step = LogRampStep(targetPWM, currentPWM);
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

float data_class::Calcu_fm2000_motor_pwm(uint16_t adc_value, uint8_t fnr, uint8_t &confirmed_dir, float &current_pwm, uint8_t &dir_change_wait_cnt)
{
	uint8_t request_dir = 0;
	float target_pwm = 0.0f;
	const float stop_threshold = 0.01f;
	const uint8_t direction_change_wait_10ms = 50;

	if(fnr == 1) {
		request_dir = 1;
		target_pwm = adc_to_pwm(adc_value);
	}
	else if(fnr == 2) {
		request_dir = 2;
		target_pwm = -adc_to_pwm(adc_value);
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
				target_pwm = (request_dir == 1) ? adc_to_pwm(adc_value) : -adc_to_pwm(adc_value);
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

	current_pwm = PWM_UpdateRoutine(target_pwm, current_pwm, sysFlag.stop_throttle);
	return current_pwm;
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

	inputRaw.source_pwm.f1 = (fm2000_gpio.FNR1 == 1) ? adc_to_pwm(ladcValue[0]) :
	                         (fm2000_gpio.FNR1 == 2) ? -adc_to_pwm(ladcValue[0]) : 0.0f;
	inputRaw.source_pwm.f2 = (fm2000_gpio.FNR2 == 1) ? adc_to_pwm(ladcValue[1]) :
	                         (fm2000_gpio.FNR2 == 2) ? -adc_to_pwm(ladcValue[1]) : 0.0f;

	inputRaw.target_pwm.f1 = Calcu_fm2000_motor_pwm(ladcValue[0], fm2000_gpio.FNR1, fm2000_dir1, currentPWM1, fm2000_dir_change_wait1);
	inputRaw.target_pwm.f2 = Calcu_fm2000_motor_pwm(ladcValue[1], fm2000_gpio.FNR2, fm2000_dir2, currentPWM2, fm2000_dir_change_wait2);
	inputRaw.source_pwm.f1 = (fm2000_dir1 == 1) ? adc_to_pwm(ladcValue[0]) :
	                         (fm2000_dir1 == 2) ? -adc_to_pwm(ladcValue[0]) : 0.0f;
	inputRaw.source_pwm.f2 = (fm2000_dir2 == 1) ? adc_to_pwm(ladcValue[1]) :
	                         (fm2000_dir2 == 2) ? -adc_to_pwm(ladcValue[1]) : 0.0f;
	return;
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
	//pPWM->PWM_1ms();

}

void data_class::force_pwm_off_for_over_current()
{
	inputRaw.target_pwm.f1 = 0.0f;
	inputRaw.target_pwm.f2 = 0.0f;
	inputRaw.source_pwm.f1 = 0.0f;
	inputRaw.source_pwm.f2 = 0.0f;
	currentPWM1 = 0.0f;
	currentPWM2 = 0.0f;
	if(pPWM != 0) {
		pPWM->Update_PWM(1, 0.0f, 0.0f);
		pPWM->brake_on_flag = 0;
	}
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_RESET);
}

void data_class::ten_millisec_routine()
{

	//printf("ten_millisec_routine can_TimeOut[%d]\r\n",pCAN->can_TimeOut);
	Get_AdcData();
	update_current();
	if(over_current_protect) {
		if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_9_OVER_CURRENT);
		return;
	}

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

	if(batt.measure_m12_current >= MAX_CURRENT) {
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

	if(fet_temp_protect) {
		if(pFND595 != 0) pFND595->PrintDigit(ERROR_CODE_3_FET_OVER_TEMPERATURE);
		force_pwm_off_for_over_current();
		return;
	}
	local_lift=Get_localGPIO();//can과 상관없이 처리

	 if(pCAN->can_TimeOut>0)pCAN->can_TimeOut--;
	 if(pCAN->can_TimeOut){
		sysFlag.canReady = 1;
	}
	else {
		sysFlag.canReady = 0;
		ON_Board_INPUT(); // CAN 없을 때 로컬 입력(스위치/포텐쇼/리밋) 사용
	}

	if(current_state != prev_state) {
		state_change_flag_timeout = STABILIZE_DELAY_10MS_TICK;
	    prev_state = current_state; // 상태 변경 감지
	}

//++++++++++++
	float fet_temp_pwm_scale = get_fet_temp_pwm_scale();
	pPWM->Update_PWM(1,
			inputRaw.target_pwm.f1 * fet_temp_pwm_scale,
			inputRaw.target_pwm.f2 * fet_temp_pwm_scale);
	pPWM->brake_on_flag=0;

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

	HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);
}

void data_class::hnd_millisec_routine(){
	if(sysFlag.SaveEEPROM){
		sysFlag.SaveEEPROM=0;
		printf("###SaveEEPROM ignored: SYSTEM_CONF disabled===\r\n");
	}
	if(pDataClass->sysFlag.canReady)CAN_DCU_Information();
}

void data_class::onesec_routine()
{
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
	batt.measure_battery_voltage=get_voltage(ladcValue[7]);
	update_error_code_once_per_second();
	display_error_code_once_per_second();

	printf("FNR1[%d] FNR2[%d] error[%d] [%.1fV/%.2fA] FT[%d] src[%.2f,%.2f] tgt[%.2f,%.2f] [%lu,%lu] ADC[%d,%d,%d,%d]\r\n",
			fm2000_gpio.FNR1,
			fm2000_gpio.FNR2,
			error_code.code,
			batt.measure_battery_voltage,
			batt.measure_m12_current,
			batt.fet_temp,
			inputRaw.source_pwm.f1,
			inputRaw.source_pwm.f2,
			inputRaw.target_pwm.f1,
			inputRaw.target_pwm.f2,
			ladcValue[2],ladcValue[3],
			ladcValue[6],ladcValue[7],ladcValue[8],ladcValue[9]);


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
	uint32_t Pub_ID=0x588;
	uint8_t buf[8]={0,};
	uint8_t pack_buf[7]={0,};

	// 전송용 RPM 보정: 전진/후진 동일 배율
	// 제어 변수 pPWM->rpm 은 변경하지 않음
	int16_t rpm1 = (int16_t)(pPWM->rpm.f1 * 1.667f);
	int16_t rpm2 = (int16_t)(pPWM->rpm.f2 * 1.667f);
	uint8_t current=(uint16_t)batt.measure_m12_current;//unit A
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
	ldcu.id=ZIGBEE_MY_ID;
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
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc[i]=HAL_ADC_GetValue(&hadc1);
		//ladcValue[i]=HAL_ADC_GetValue(&hadc1);
	}
	memcpy(ladcValue,adc,32);
	ladcValue[0]=get_m0_filter(adc[0]);
	ladcValue[1]=get_m1_filter(adc[1]);
	ladcValue[2]=get_m2_filter(adc[2],0.05f);//over current detect
	ladcValue[3]=get_m3_filter(adc[3],0.05f);//over current detect
	ladcValue[6]=get_m6_filter(adc[6],0.1f);//wcs current detect
	ladcValue[8]=get_m8_filter(adc[8],0.05f);//bemf detect
	ladcValue[9]=get_m9_filter(adc[9],0.05f);//bemf detect
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
    const float SENSITIVITY = 0.0187f;
    const uint16_t wcs_zero_adc = 1938;//2068;

    float zero_voltage = ((float)wcs_zero_adc * VREF) / ADC_MAX;
    float voltage = ((float)adc * VREF) / ADC_MAX;

    return (voltage - zero_voltage) / SENSITIVITY;
}

void data_class::update_current()
{
	batt.measure_m12_current = get_wcs1600_current(ladcValue[6]);
	batt.measure_m1_current = batt.measure_m12_current;
	batt.measure_m2_current = batt.measure_m12_current;
}

void data_class::update_error_code_once_per_second()
{
	float measured_voltage = batt.measure_battery_voltage;
	float low_voltage = MIN_VOLTAGE;
	float over_voltage = MAX_VOLTAGE;

	memset(&error_code,0,sizeof(ERROR_CODE_STATE));
	error_code.low_voltage = (measured_voltage < low_voltage);
	error_code.over_voltage = (measured_voltage > over_voltage);
	error_code.fet_over_temperature = (fet_temp_protect || (int8_t)batt.fet_temp >= FET_TEMP_DERATE_START);
	error_code.motor_over_temperature = ((int8_t)batt.motor_temp > LIMIT_MOTOR_TEMP);
	error_code.motor1_fault = batt.moter_error.m1_error;
	error_code.motor2_fault = batt.moter_error.m2_error;
	error_code.tbd1 = 0;
	error_code.tbd2 = 0;
	error_code.over_current = (over_current_protect || batt.measure_m12_current >= MAX_CURRENT);

	if(error_code.over_current) error_code.code = ERROR_CODE_9_OVER_CURRENT;
	else if(error_code.fet_over_temperature) error_code.code = ERROR_CODE_3_FET_OVER_TEMPERATURE;
	else if(error_code.low_voltage) error_code.code = ERROR_CODE_1_LOW_VOLTAGE;
	else if(error_code.over_voltage) error_code.code = ERROR_CODE_2_OVER_VOLTAGE;
	else if(error_code.motor_over_temperature) error_code.code = ERROR_CODE_4_MOTOR_OVER_TEMPERATURE;
	else if(error_code.motor1_fault) error_code.code = ERROR_CODE_5_MOTOR1_FAULT;
	else if(error_code.motor2_fault) error_code.code = ERROR_CODE_6_MOTOR2_FAULT;
	else if(error_code.tbd1) error_code.code = ERROR_CODE_7_TBD1;
	else if(error_code.tbd2) error_code.code = ERROR_CODE_8_TBD2;
	else {
		error_code.code = ERROR_CODE_0_NORMAL;
		error_code.normal = 1;
	}
}

void data_class::display_error_code_once_per_second()
{
	if(pFND595 == 0) return;

	if(error_code.code != ERROR_CODE_0_NORMAL) {
		pFND595->PrintDigit(error_code.code);
		return;
	}

	uint8_t pattern = 0;

	if(fm2000_gpio.FNR2 == 1) pattern |= FND595::SEG_B;
	else if(fm2000_gpio.FNR2 == 2) pattern |= FND595::SEG_C;

	if(fm2000_gpio.FNR1 == 1) pattern |= FND595::SEG_F;
	else if(fm2000_gpio.FNR1 == 2) pattern |= FND595::SEG_E;

	if(pattern == 0) pFND595->PrintDigit(0);
	else pFND595->WriteRaw(pattern);
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
