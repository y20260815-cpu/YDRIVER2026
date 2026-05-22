/*
 * eps.cpp
 *
 *  Created on: Jun 1, 2025
 *      Author: thpark
 */
#include "extern.h"
#include "eps.h"
eps *pEPS;

eps::eps() {
	// TODO Auto-generated constructor stub

}

eps::~eps() {
	// TODO Auto-generated destructor stub
}

XY_POSITION eps::get_Angle_From_switch(uint8_t junhujin, uint8_t lr_stop, uint8_t en_spin){
	XY_POSITION xyPos;
	xyPos.x=0;
	xyPos.y=0;
	switch(junhujin)
	{
		case 0://정지
			if(en_spin)//spin
			{
				switch(lr_stop){
				case 1://left spin
					xyPos.x=-1;
					xyPos.y=0;
					break;
				case 2://right spin
					xyPos.x=1;
					xyPos.y=0;
					break;
				default:
					xyPos.x=0;
					xyPos.y=0;
					break;
				}
				break;
			}
			xyPos.x=0;
			xyPos.y=0;
			break;
		case 1://전진
			xyPos.x=0;
			xyPos.y=1;
			break;
		case 2:
			xyPos.x=0;
			xyPos.y=-1;
			break;
	}
	return xyPos;
}


MOTOR_DIRECTION eps::set_Angle_To_motor(XY_POSITION new_xy_pos){
	MOTOR_DIRECTION motor;

//	if(new_xy_pos.y==0){}
//	else if(new_xy_pos.y==1){}
//	else if(new_xy_pos.y==-1){}
//	else if(new_xy_pos.y==2){}
//	else if(new_xy_pos.y==-2){}
	//전후진
	if(new_xy_pos.x==0){
		switch(new_xy_pos.y){
		case 0:
			break;
		case 1:
			motor.m1_dir=0;
			motor.m2_dir=0;
			break;
		case -1:
			motor.m1_dir=1;
			motor.m2_dir=1;
			break;
		}
	}
	else{
		switch(new_xy_pos.x){
		case 0:
			break;
		case 1:
			motor.m1_dir=0;
			motor.m2_dir=1;
			break;
		case -1:
			motor.m1_dir=1;
			motor.m2_dir=0;
			break;
		}
	}
	return motor;
}

void eps::get_motor_direction()
{
#if 0
	uint8_t _brake_rate=5;
	uint8_t motor_dir1=pDataClass->sysFlag.motor_dir1;
	uint8_t motor_dir2=pDataClass->sysFlag.motor_dir2;

	if( pDataClass->CHANGE_FLAG==0) {
		//ipwm=Button_switch_Event(ipwm.pwm1, ipwm.pwm2);
		return;
	}
	//printf("#@#Toggle_Switch_Event[%d]  direction_evt[%d] ipwm[%d][%d]\r\n", pDataClass->CHANGE_FLAG, pDataClass->direction_evt, ipwm.pwm1, ipwm.pwm2);
	//ipwm.pwm1=pDataClass->gMAIN.ex_pwm1-_brake_rate;
	//ipwm.pwm2=pDataClass->gMAIN.ex_pwm2-_brake_rate;

	if(ipwm.pwm1<10 && ipwm.pwm2<10) {
		  pDataClass->gMAIN.flg_state.stop1=0;
		  pDataClass->CHANGE_FLAG=0;
		 // printf("#@#pwm (2)EVENT_Activity [%d] \r\n", pDataClass->direction_evt);

		  switch( pDataClass->direction_evt){
		  		  case EVT_SW1_CENTER:
		  			  motor_dir1=0;
		  			  motor_dir2=0;
		  			  break;
		  		  case EVT_SW1_JENJIN:
		  			  motor_dir1=0;
		  			  motor_dir2=0;
		  			  break;
		  		  case EVT_SW1_HUJIN:
		  			  motor_dir1=1;
		  			  motor_dir2=1;
		  			  break;
		  		 case EVT_SW2_CENTER_SPIN:
		  			  switch(pDataClass->gMAIN.iPort.JENHUJIN1){
	  				  	  case CENTER:
							motor_dir1=0;
							motor_dir2=0;
	  					  break;
		  				  case JENJIN:
		  		  			  motor_dir1=0;
		  		  			  motor_dir2=0;
		  					  break;
		  				  case HUJIN:
		  		  			  motor_dir1=1;
		  		  			  motor_dir2=1;
		  					  break;
		  			  }
		  			 break;
		  		  case EVT_SW2_LEFT_SPIN:
		  			  motor_dir1=0;
		  			  motor_dir2=1;
		  			  break;
		  		  case EVT_SW2_RIGHT_SPIN:
		  			  motor_dir1=1;
		  			  motor_dir2=0;
		  			  break;
		  }

		  pDataClass->sysFlag.motor_dir1=motor_dir1;
		  pDataClass->sysFlag.motor_dir2=motor_dir2;
	}
#endif
}
