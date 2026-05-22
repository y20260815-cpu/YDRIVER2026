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

	gMAIN.flg_state.trot_zero=1;
	relayAllOFF();
}

data_class::~data_class()
{
	// TODO Auto-generated destructor stub
}

void data_class::power_on(){
	PWR_ON_Flg=1;
	float battery_table[6]={12.0f, 24.0f, 36.0f,48.0f, 60.0f, 72.0f};
	memset(&inputRaw,0,sizeof(POSITION_ANGLE));
	gMAIN.relay.MC2=1;
	HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	gMAIN.relay.MC1=1;
	HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_SET);
	pPWM->brake_on_flag=1;

	motor1_polarity=pDataClass->motor1_polarity;
	motor2_polarity=pDataClass->motor2_polarity;

	accel_rate=setup_data.conf2.accel/100.0f;
	decel_rate=setup_data.conf2.decel/100.0f;
	gamsok_idx=setup_data.conf2.brake_rate;
	brake_delay=setup_data.conf2.brake_delay;//
	set_foreward=1.0f;//setup_data.conf2.foreward/100.0f;
	set_backward=1.0f;//setup_data.conf2.backward/100.0f;
	batt.use_battery_voltage=battery_table[setup_data.conf1.battery_voltage];
	batt.motor_spec_voltage =battery_table[setup_data.conf1.battery_voltage];
	//batt.motor_spec_rpm=MOTOR_MAX_RPM;
	pidCONF.rpm_to_pwm_scale=1.0f/MOTOR_MAX_RPM;
	pidCONF.Kp=(float)setup_data.conf2.stop_slip/100.0f;


	//pPWM->pid_k
    //setup_data.conf1.motor1_polarity = 1;  // 0 → 1로 변경
    //setup_data.conf1.motor2_polarity = 1;  // 0 → 1로 변경
}

void data_class::power_off(){
	PWR_ON_Flg=0;
	HAL_GPIO_WritePin(pSD_GPIO_Port, pSD_Pin, GPIO_PIN_RESET);//FET ALL OFF
	gMAIN.relay.MC1=0;
	HAL_GPIO_WritePin(pRY1_GPIO_Port, pRY1_Pin, GPIO_PIN_RESET);//Main Relay Off
	gMAIN.relay.MC2=0;
	HAL_GPIO_WritePin(pRY2_GPIO_Port, pRY2_Pin, GPIO_PIN_RESET);//Aux Relay Off
	pPWM->brake_on_flag=1;
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
    // 오차에 따른 상수 선택
//    float k = (error > 0) ? accel_rate : decel_rate;
    float k = (error > 0) ? accel_rate : decel_rate;
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

DoubleF_VALUE data_class::Calcu_sourcePWM(float i_poten, float limit, uint8_t jenhujin, uint8_t stop_lr){
	DoubleF_VALUE o_poten;
	float control_value=Calcu_limit(i_poten, limit);
	o_poten.f1=o_poten.f2=control_value;
//	stabilize_change();
	if(jenhujin){
		switch(stop_lr){
			case 3: break;
			case 0:	o_poten.f1 =o_poten.f2 =0;break;//ALL press
			case 1:	o_poten.f1 *=gamsok_ratio[gamsok_idx];break;
			case 2:	o_poten.f2 *=gamsok_ratio[gamsok_idx];break;
			default:break;
		}
	}
	else//board_io.toggle.jenhujin==0 중립일때는 SPIN
	{
		switch(stop_lr)
		{
			case 3:	case 0:	o_poten.f1 =o_poten.f2 =0;break;//ALL press
			case 1:	o_poten.f1 *=(-1.0f);o_poten.f2 *=(1.0f);break;//spin left?
			case 2:	o_poten.f1 *=(1.0f); o_poten.f2 *=(-1.0f);break;//spin right
			default:break;
		}
	}
	//cprintf(C_RED, "i_poten[%.1f] limit[%.1f] control_value[%.1f] o_poten[%.1f|%.1f] jenhujin[%d] stop_lr[%d] gamsok_ratio[%.1f]\r\n",i_poten, limit, control_value, o_poten.f1, o_poten.f2,jenhujin, stop_lr, gamsok_ratio[gamsok_idx]);
	return o_poten;
}


void data_class::ON_Board_INPUT()
{
	DoubleF_VALUE imsi;
	read_in_port();
	float limit=ladcValue[1]/4096.0f;
	float adj_vr=scale_value(ladcValue[0], 1013, 3179);//측정한값
	//ctl_pwm.source_pwm=Calcu_sourcePWM(adj_vr, limit);
	inputRaw.source_pwm=Calcu_sourcePWM(adj_vr, limit, board_io.toggle.jenhujin, board_io.toggle.t_sw);
	inputRaw.rpm.f1=pPWM->rpm.f1;
	inputRaw.rpm.f2=pPWM->rpm.f2;
	if((ex_board_io.toggle.u8 != board_io.toggle.u8)){
		JENHUJIN_SWITCH_CHANGE_FLAG=1;
		printf("\r\n(#1111)CHANGE_EVT |board_io[%d] |ctl_pwm.io[%d] \r\n",board_io.toggle.jenhujin, inputRaw.io.toggle.jenhujin);
	}
	ex_board_io=board_io;

	if(JENHUJIN_SWITCH_CHANGE_FLAG){
		printf("@@@@@@@CHANGE_FLAG@@@@@@@@@@\r\n");
		inputRaw.source_pwm.f1 =0;
		inputRaw.source_pwm.f2 =0;
		if(fabs(inputRaw.target_pwm.f1)<0.2f && fabs(inputRaw.target_pwm.f2)<0.2f)
		{
			inputRaw.io.toggle=board_io.toggle;
			JENHUJIN_SWITCH_CHANGE_FLAG=0;
		}
	}
	currentPWM1 = PWM_UpdateRoutine(inputRaw.source_pwm.f1, currentPWM1, sysFlag.stop_throttle);
	currentPWM2 = PWM_UpdateRoutine(inputRaw.source_pwm.f2, currentPWM2, sysFlag.stop_throttle);

	imsi.f1=currentPWM1;
	imsi.f2=currentPWM2;
	switch(inputRaw.io.toggle.jenhujin){
		case 0:
			break;
		case 1:
			break;
		case 2://역방향
			imsi.f1 *=-1.0f;
			imsi.f2 *=-1.0f;
			break;
		default:
			break;
	}
	inputRaw.target_pwm.f1=imsi.f1;
	inputRaw.target_pwm.f2=imsi.f2;
	//printf("evt[%d] ctl_pwm.io.toggle.jenhujin[%d] target_pwm.fpwm1[%.1f] currentPWM[%.1f/%.1f] source_pwm[%.1f] rpm[%.1f/%.1f]\r\n", JENHUJIN_SWITCH_CHANGE_FLAG, ctl_pwm.io.toggle.jenhujin, ctl_pwm.target_pwm.fpwm1, currentPWM1, currentPWM2, ctl_pwm.source_pwm.fpwm1, ctl_pwm.rpm.fpwm1, ctl_pwm.rpm.fpwm2);
}

LIFT_BUTTON data_class::Get_localGPIO(){
	LIFT_BUTTON btn;
	uint8_t rdata=0;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_AP_GPIO_Port, pIO_AP_Pin));
		rdata<<=1;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_AN_GPIO_Port, pIO_AN_Pin));
		rdata %=3;
		if(sysFlag.canReady) btn.lift_WudLimit=rdata;
		else{
		//TBD for FNR switch
		}

		rdata=0;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_BP_GPIO_Port, pIO_BP_Pin));
		rdata<<=1;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_BN_GPIO_Port, pIO_BN_Pin));
		rdata %=3;
		btn.lift_Wud=rdata;

		rdata=0;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_CP_GPIO_Port, pIO_CP_Pin));
		rdata<<=1;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_CN_GPIO_Port, pIO_CN_Pin));
		rdata %=3;
		btn.lift_Xud=rdata;
	return btn;
}

POSITION_ANGLE data_class::CAN_Board_INPUT(bool isCan)
{
	POSITION_ANGLE ret;
	DoubleF_VALUE imsi;

	if(!isCan){
		memset(&ret,0,sizeof(POSITION_ANGLE));
		return ret;
	}
	motor1_polarity=0;
	motor2_polarity=0;
	brake_delay=vcu_sdu.canBrakeDelay;
	batt.use_battery_voltage=vcu_sdu.canBattery;
	batt.motor_spec_voltage =vcu_sdu.canBattery;

	ret.io.btn=vcu_sdu.btn;
	ret.io.toggle=vcu_sdu.toggle;
	ret.source_pwm.f1=(vcu_sdu.LeftMotor_velocity/1000.0f);
	ret.source_pwm.f2=(vcu_sdu.RightMotor_velocity/1000.0f);

	ret.rpm.f1=pPWM->rpm.f1;
	ret.rpm.f2=pPWM->rpm.f2;
	imsi=ret.source_pwm;

	ret.target_pwm=imsi;
	//cprintf(C_YELLOW, "source_pwm[%.1f^%.1f] gamsok_ratio[%.1f] target_pwm[%.1f^%.1f]\r\n",ret.source_pwm.f1, ret.source_pwm.f2, gamsok_ratio[gamsok_idx], ret.target_pwm.f1, ret.target_pwm.f2);
	return ret;
}


void data_class::one_millisec_routine()
{
	pPWM->PWM_1ms();
}

void data_class::can_process_routine(){
		inputRaw=CAN_Board_INPUT(true);
        HOLD_Emergency = inputRaw.io.btn.emergency;
}

void data_class::ten_millisec_routine()
{

	//printf("ten_millisec_routine can_TimeOut[%d]\r\n",pCAN->can_TimeOut);
	local_lift=Get_localGPIO();//can과 상관없이 처리

	 if(pCAN->can_TimeOut>0)pCAN->can_TimeOut--;
	 if(pCAN->can_TimeOut){
		sysFlag.canReady = 1;
	}
	else {
		sysFlag.canReady = 0;
		inputRaw.source_pwm.f1=0;
		inputRaw.source_pwm.f2=0;
	}

	if(current_state != prev_state) {
		state_change_flag_timeout = STABILIZE_DELAY_10MS_TICK;
	    prev_state = current_state; // 상태 변경 감지
	}

//++++++++++++
	float stop_pwm1=fabs(inputRaw.source_pwm.f1);
	float stop_pwm2=fabs(inputRaw.source_pwm.f2);
	pPWM->Update_PWM(0, inputRaw.target_pwm.f1, inputRaw.target_pwm.f2);
	pPWM->CheckBrakeState(stop_pwm1, stop_pwm2);

//liftControl++
	if(pPWM->brake_on_flag && (inputRaw.io.btn.ioMsg==0))
	{
		LIFT_BUTTON llift;
		//printf("lift_control lift_Wud[%d] lift_Xud[%d]\r\n",local_lift.lift_Wud, local_lift.lift_Xud);
		llift.lift_Wud=(local_lift.lift_Wud)?local_lift.lift_Wud:inputRaw.io.btn.lift_Wud;
		if(local_lift.lift_WudLimit==1 && llift.lift_Wud==1) llift.lift_Wud=0;
		if(local_lift.lift_WudLimit==2 && llift.lift_Wud==2) llift.lift_Wud=0;

		llift.lift_Xud=(local_lift.lift_Xud)?local_lift.lift_Xud:inputRaw.io.btn.lift_Xud;
		lift_control(llift.lift_Xud, llift.lift_Wud);
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

	if(PWR_ON_Flg){
		if(pPWM->brake_on_flag) HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(pRY3_GPIO_Port, pRY3_Pin, GPIO_PIN_RESET);
}

void data_class::hnd_millisec_routine(){
	if(sysFlag.SaveEEPROM){
		sysFlag.SaveEEPROM=0;
		memcpy(&pDataClass->setup_data, &pCAN->setup_data, sizeof(CONFIG_TOTAL));
		pFlash_mem->save_to_flash_config(pDataClass->setup_data);
		printf("###SaveEEPROM===\r\n");
	}
	if(pDataClass->sysFlag.canReady)CAN_DCU_Information();
}

void data_class::onesec_routine()
{
#if 0
	uint16_t one_sec_adc[16] = {0};
	
	// === ADC 하드웨어 진단 START ===
	printf("\n=== ADC Hardware Diagnostic ===\n");
	
	// 1. 내부 온도 센서 테스트 (ADC는 정상 동작하는지 확인)
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	
	// Discontinuous 모드 일시 비활성화
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	HAL_ADC_Init(&hadc1);
	
	// 1-1. 내부 온도센서 테스트
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		uint16_t temp_sensor = HAL_ADC_GetValue(&hadc1);
		printf("Internal TEMP sensor: %d (ADC OK if ~1400-1600)\n", temp_sensor);
	} else {
		printf("ERROR: Internal TEMP sensor timeout!\n");
	}
	HAL_ADC_Stop(&hadc1);
	
	// 1-2. 내부 기준전압(VREF) 테스트
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
		uint16_t vref = HAL_ADC_GetValue(&hadc1);
		printf("Internal VREF: %d (should be ~1200-1500)\n", vref);
	} else {
		printf("ERROR: Internal VREF timeout!\n");
	}
	HAL_ADC_Stop(&hadc1);
	
	printf("=== External Channels Test ===\n");
	
	// 2. 외부 채널 테스트
	uint32_t channels[10] = {
		ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4,
		ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_14, ADC_CHANNEL_15
	};
	
	const char* ch_names[10] = {
		"CH0(PA0)", "CH1(PA1)", "CH2(PA2)", "CH3(PA3)", "CH4(PA4-FET_TEMP)",
		"CH5(PA5-MOT_TEMP)", "CH6(PA6)", "CH7(PA7-VBAT)", "CH14(PC4)", "CH15(PC5)"
	};
	
	for(int i=0; i<10; i++){
		sConfig.Channel = channels[i];
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
		
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
			one_sec_adc[i] = HAL_ADC_GetValue(&hadc1);
			
			// 문제 진단
			if(one_sec_adc[i] == 0) {
				printf("%s: %d [WARN: GND or disconnected]\n", ch_names[i], one_sec_adc[i]);
			} else if(one_sec_adc[i] >= 4090) {
				printf("%s: %d [WARN: VCC or open circuit]\n", ch_names[i], one_sec_adc[i]);
			} else {
				printf("%s: %d [OK]\n", ch_names[i], one_sec_adc[i]);
			}
		} else {
			printf("%s: TIMEOUT [ERROR]\n", ch_names[i]);
			one_sec_adc[i] = 0;
		}
		HAL_ADC_Stop(&hadc1);
	}
	
	// 원래 설정으로 복구
	hadc1.Init.DiscontinuousConvMode = ENABLE;
	hadc1.Init.NbrOfDiscConversion = 1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.NbrOfConversion = 10;
	HAL_ADC_Init(&hadc1);
	
	printf("=================================\n\n");
#endif
	Get_AdcData();
	batt.fet_temp=get_ntc_temperature(ladcValue[4]);
	batt.motor_temp=(int8_t)get_ntc_temperature(ladcValue[5]);
	if(pPWM->brake_on_flag)//No EMB_RelayOn
	{
		batt.measure_emb_resister=(float)get_emb_resister(ladcValue[7], ladcValue[6]);
	}
	batt.measure_battery_voltage=(float)get_voltage(ladcValue[7]);
	batt.measure_m1_current=(float)(ladcValue[8]);
	batt.measure_m2_current=(float)(ladcValue[9]);
	batt.measure_m12_current = fmaxf(fabs(batt.measure_m1_current), fabs(batt.measure_m2_current));
	batt.measure_m12_current -=2715.0f;
#if 1
//printf("Potentio_val[%04d] limit[%04d] toggle[%02d] mi_dir[%d] m2_dir[%d]\r\n", Potentio_val, vcu_sdu.limit, vcu_sdu.toggle.u8, sysFlag.motor_dir1, sysFlag.motor_dir2);
	printf("pid_k[%.2f] accel[%04d] decel[%04d] brake_delay[%04d] brake_rate[%04d] use_battery_voltage[%.1f] measure_emb_resister[%.1f] current[%.1f+%.1f=%.1f]\r\n",
			pidCONF.Kp,  setup_data.conf2.accel, setup_data.conf2.decel,brake_delay, setup_data.conf2.brake_rate,
			batt.use_battery_voltage,batt.measure_emb_resister, batt.measure_m1_current, batt.measure_m2_current, batt.measure_m12_current);
//	printf("ex_pwm1[%04d] ex_pwm1[%04d] state[%d]\r\n",gMAIN.ex_pwm1,  gMAIN.ex_pwm2, gMAIN.flg_state.u8);
	printf("HOLD_Emergency[%d] emb_delay[%d] forward[%.1f^%.1f]\r\n", HOLD_Emergency,brake_delay, set_foreward, set_backward);
	printf("[BUTTON] JENHUJIN1[%d] stopBTN_lr[%d] lift_Xud[%d] lift_Wud[%d]\r\n",board_io.toggle.jenhujin, board_io.toggle.t_sw, board_io.btn.lift_Xud, board_io.btn.lift_Wud);
	printf("inport[%x] stop[%d][%d] \r\n", board_io.toggle.u8,  gMAIN.flg_state.stop1, gMAIN.flg_state.stop2);
	printf("fet_temp[%d/%d] motor_temp[%d/%d] battery_voltage[%.f|%d]\r\n",batt.fet_temp,ladcValue[4], batt.motor_temp,ladcValue[5], batt.measure_battery_voltage, ladcValue[7]);
	printf("vr[%.2f/%.2f] rpm[%.1f|%.1f] target_rpm[%.1f|%.1f] eRPM[%.1f|%.1f] \r\n",
			inputRaw.source_pwm.f1, inputRaw.source_pwm.f2, inputRaw.rpm.f1, inputRaw.rpm.f2,pPWM->target_rpm.f1, pPWM->target_rpm.f2, pPWM->error_rpm.f1, pPWM->error_rpm.f2);
#endif
	relay_control();
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
	uint32_t Pub_ID=0x5B8;
	uint8_t buf[8]={0,};
	uint8_t pack_buf[7]={0,};

	int16_t rpm1=(int16_t)pPWM->rpm.f1;//(uint8_t)(fabs(inputRaw.rpm.f1)/100.0f);
	int16_t rpm2=(int16_t)pPWM->rpm.f2;//(uint8_t)(fabs(inputRaw.rpm.f2)/100.0f);
	uint8_t current=(uint16_t)batt.measure_m12_current/16;
	uint16_t batt10=(uint16_t)batt.measure_battery_voltage+5;//0.5V보정
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
		upFlag.limitSwitch=local_lift.lift_Xud;//방제차-limitSwitch
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
	for(int i=0;i<10;i++){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		ladcValue[i]=HAL_ADC_GetValue(&hadc1);
	}
}

uint16_t data_class::get_voltage(uint16_t value)
{
	uint16_t voltage;
	float t = (float)value * 3.3f / 0xfff; // 읽은 센서값을 전압으로 변경
    t=t*520;//51K 1K x10
    voltage=(uint16_t)t;
    return voltage;
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

uint16_t data_class::get_m2_filter(uint16_t adc)
{
  static float m2_value;
  m2_value=(m2_value*(1-ADC_SENSITIVITY))+(adc*ADC_SENSITIVITY);
  return (uint16_t)m2_value;
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
	switch(port){
	case SW_JENHUJIN:
		rdata|=!!(HAL_GPIO_ReadPin(pIO_AP_GPIO_Port, pIO_AP_Pin));
		rdata<<=1;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_AN_GPIO_Port, pIO_AN_Pin));
		rdata %=3;
		break;
	case SW_STOP:
		rdata|=!!(HAL_GPIO_ReadPin(pIO_BP_GPIO_Port, pIO_BP_Pin));
		rdata<<=1;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_BN_GPIO_Port, pIO_BN_Pin));
		rdata %=4;
		break;
	case SW_LIFT:
		rdata|=!!(HAL_GPIO_ReadPin(pIO_CP_GPIO_Port, pIO_CP_Pin));
		rdata<<=1;
		rdata|=!!(HAL_GPIO_ReadPin(pIO_CN_GPIO_Port, pIO_CN_Pin));
		rdata %=3;
		break;
	case SW_BISANG:
		break;
	}
	return rdata;
}

void data_class::read_in_port(){

	uint8_t rdata=0;
	board_io.toggle.jenhujin=get_switch_port(SW_JENHUJIN);
	board_io.toggle.t_sw=get_switch_port(SW_STOP);
	//board_io.btn.lift_Xud=get_switch_port(SW_LIFT);

	rdata=0;
	rdata|=!!(HAL_GPIO_ReadPin(pIO_DP_GPIO_Port, pIO_DP_Pin));
	if(rdata==1) {
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(pIO_DP_GPIO_Port, pIO_DP_Pin)) HOLD_Emergency=1;
	}
}

int data_class::clamp(int value, int min, int max) {
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}
