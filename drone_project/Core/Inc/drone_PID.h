/*
 * drone_PID.h
 *
 *  Created on: Jul 24, 2025
 *      Author: Robert Chu
 */

#ifndef INC_DRONE_PID_H_




#include <stdint.h> //uint16_t

//typedefs
#ifndef float32_t
typedef float float32_t; //arm_math.h also defines float32_t as this
#endif

typedef struct
{
	//cannot set values here since no memory allocated in declaration
	float Ts; //sampling time
	float proportional_gain;
	float integral_gain;
	float derivative_gain;

	float tau; // 1/(cutoff frequency of low pass filter after derivative component)

	float proportional_out;
	float integral_out;
	float derivative_out;

	float32_t error; //error is signed
	float32_t measured_pos;

	float total_out;
	float out_max;
	float out_min;
} pid_controller;





//PID control related function prototypes
void initialize_PID(pid_controller *controller, uint16_t updated_measured_pos);
void set_gains_PID(pid_controller *controller, float Kp, float Ki, float Kd);
void update_PID(pid_controller *controller, float updated_measured_pos, float set_point);
void update_motor_input(int16_t new_out, uint32_t **active_buffer, uint32_t **inactive_buffer);






#define INC_DRONE_PID_H_



#endif /* INC_DRONE_PID_H_ */
