/*
 * drone_PID.h
 *
 *  Created on: Jul 24, 2025
 *      Author: Robert Chu
 */

#ifndef INC_DRONE_PID_H_




#include <stdint.h> //uint16_t
#include "stm32f1xx_hal.h"  //timer for PWM signal generation
//typedefs
#ifndef float32_t
typedef float float32_t; //arm_math.h also defines float32_t as this
#endif

typedef struct
{
	//cannot set values here since no memory allocated in declaration
	float32_t Ts; //sampling time
	float32_t proportional_gain;
	float32_t integral_gain;
	float32_t derivative_gain;

	float32_t tau; // 1/(cutoff frequency of low pass filter after derivative component)

	float32_t proportional_out;
	float32_t integral_out;
	float32_t derivative_out;

	float32_t error; //error is signed
	float32_t measured_pos;

	float32_t total_out;
	float32_t out_max;
	float32_t out_min;
} pid_controller;

typedef struct
{
	pid_controller *pitch_PID_controller;
	pid_controller *roll_PID_controller;

	float32_t thrust_signal;
	float32_t yaw_signal;
	float32_t pitch_signal;
	float32_t roll_signal;

	//motor PWM outputs, values can only be between [0,1000], still the type is int because motor mixing may cause values to fall below 0 that cause overflow error
	int16_t motor1_total_out; //Front Left
	int16_t motor2_total_out; //Front Right
	int16_t motor3_total_out; //Rear Left
	int16_t motor4_total_out; //Rear Right
	int16_t motor_out_max;
	int16_t motor_out_min;
} drone_motor_controller;





//PID controller related function prototypes
void initialize_PID(pid_controller *controller, float32_t updated_measured_pos);
void set_gains_PID(pid_controller *controller, float32_t Kp, float32_t Ki, float32_t Kd);
void update_PID(pid_controller *controller, float32_t updated_measured_pos, float32_t set_point);


//drone_motor_controller functions
void initialize_drone_motor_controller(drone_motor_controller *drone_controller, pid_controller *pitch_controller, pid_controller *roll_controller);
void update_signals(drone_motor_controller *drone_controller, float32_t new_thrust_signal, float32_t new_yaw_signal);
void update_motor_input(drone_motor_controller *drone_controller, TIM_HandleTypeDef* htim2);



#define INC_DRONE_PID_H_



#endif /* INC_DRONE_PID_H_ */
