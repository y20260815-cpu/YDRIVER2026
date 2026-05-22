/*
 * eps.h
 *
 *  Created on: Jun 1, 2025
 *      Author: thpark
 */

#ifndef EPS_EPS_H_
#define EPS_EPS_H_

class eps {
public:
	eps();
	virtual ~eps();
	void get_motor_direction();
	XY_POSITION get_Angle_From_switch(uint8_t junhujin, uint8_t lr_stop, uint8_t en_spin);
	MOTOR_DIRECTION set_Angle_To_motor(XY_POSITION new_xy_pos);
	MOTOR_DIRECTION motor_dir;
	float fAngle;
};

#endif /* EPS_EPS_H_ */
