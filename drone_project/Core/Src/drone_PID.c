/*
 * drone_PID.c
 *
 *  Created on: Jul 24, 2025
 *      Author: Robert Chu
 */


#include "drone_PID.h"

//PID control functions
void initialize_PID(drone_PID_controller *controller, uint16_t initial_measured_pitch, uint16_t initial_measured_roll)
{

	controller->Ts = 0.004; //equates to 250Hz
	controller->tau = .1;

	controller->out_max = 1000;
	controller->out_min = -1000;

	controller->proportional_gain = 0;
	controller->integral_gain = 0;
	controller->derivative_gain = 0;

	controller->proportional_out = 0;
	controller->integral_out = 0;
	controller->derivative_out = 0;

	controller->motor1_total_out = 0;
	controller->motor2_total_out = 0;
	controller->motor3_total_out = 0;
	controller->motor4_total_out = 0;

	controller->error_pitch = 0;
	controller->error_roll = 0;
	controller->measured_pitch = initial_measured_pitch;
	controller->measured_roll = initial_measured_roll;
}
void set_gains_PID(drone_PID_controller *controller, float Kp, float Ki, float Kd)
{
	controller->proportional_gain = Kp;
	controller->integral_gain = Ki;
	controller->derivative_gain = Kd;
}
void update_PID(drone_PID_controller *controller, float updated_measured_value, float set_point)
{
	float adjusted_measured_value = updated_measured_value;

	float updated_error = set_point - updated_measured_value;

	//this block makes sure that if the setpoint is near boundaries (0 or 359 degrees), can still approach the setpoint
	//from the direction that has the angle measurement spike from 0 to 359 degrees or 359 to 0 degrees
	//this is done by adjusting the error term
	if (updated_measured_value > set_point + 180 && set_point < 180)
	{
		adjusted_measured_value = adjusted_measured_value - 360;
	}
	else if (updated_measured_value < set_point - 180 && set_point >= 180)
	{
		adjusted_measured_value = adjusted_measured_value + 360;
	}
	updated_error = set_point - adjusted_measured_value;

	//calculate difference for derivative term, but need to account for when motor goes from 359->0 and 0->359
	//make sure that when it goes 359->0, the difference is 1, and 0->359 is -1
				//	float32_t position_difference = updated_measured_value - controller->measured_pos;
	if (position_difference > 300) //when it goes from 0 to 359
	{
		position_difference = position_difference - 360;
	}
	else if (position_difference < -300) //when it goes from 359 to 0
	{
		position_difference = position_difference + 360;
	}

	//updated the outputs of the P, I, and D components of the controller
	controller->proportional_out = controller->proportional_gain * updated_error;
				//	controller->integral_out = controller->integral_gain * controller->Ts * (updated_error + controller->error) / 2.0 + controller->integral_out;
	controller->derivative_out = ((controller->derivative_gain * 2) * (position_difference) //
	+ (2 * controller->tau - controller->Ts) * controller->derivative_out) / (2 * controller->tau + controller->Ts);
	//note: derivative term uses measured value instead of error term to avoid kick back

//	//Deadzone for Proportional
//	if (updated_measured_value < 90.05 && updated_measured_value > 89.95)
//	{
//		controller->proportional_out = 0;
//	}
	//clamp integrator implementation
//	float integral_min, integral_max;
//	//determine integrator limits
//	if (controller->out_max > controller->proportional_out - controller->derivative_out)
//	{
//		integral_max = controller->out_max - controller->proportional_out + controller->derivative_out; //see total_out comment to see why adding derivative term instead of subtracting here
//	}
//	else
//	{
//		integral_max = 0;
//	}
//	if (controller->out_min < controller->proportional_out - controller->derivative_out)
//	{
//		integral_min = controller->out_min - controller->proportional_out + controller->derivative_out; //see total_out comment to see why adding derivative term instead of subtracting here
//	}
//	else
//	{
//		integral_min = 0;
//	}

	//get absolute error
				//	float32_t absval_error = controller->error;
	if (absval_error < 0)
	{
		absval_error = -1 * absval_error;
	}

//	if (absval_error < 5) //limit integrator even more once closer to desired angle.
//	{
//		integral_max = 600;
//		integral_min = -600;
//	}
//	if (absval_error < 3) //limit integrator even more once closer to desired angle.
//	{
//		integral_max = 300; //RED MOTOR
//		integral_min = -300;
//	}
	//clamping of integrator
//	if (controller->integral_out > integral_max)
//	{
//		controller->integral_out = integral_max;
//	}
//	else if (controller->integral_out < integral_min)
//	{
//		controller->integral_out = integral_min;
//	}

			//	//compute total output of controller
			//	controller->total_out = controller->proportional_out + controller->integral_out - controller->derivative_out; //note negative sign on derivative term, this is correct since it is on the feedback loop
			//
			//	//deadband compensation, make sure to always provide pwm that will allow motor to be spinning.
			//	if (updated_measured_value < 90.05 && updated_measured_value > 89.95)
			//	{
			//
			//	}
			//	else if (controller->total_out > 0)
			//	{
			//		controller->total_out += 100;
			//	}
			//	else if (controller->total_out < 0)
			//	{
			//		controller->total_out -= 100;
			//	}
			//
			//	//limit total output of controller
			//	if (controller->total_out > controller->out_max)
			//	{
			//		controller->total_out = controller->out_max;
			//	}
			//	else if (controller->total_out < controller->out_min)
			//	{
			//		controller->total_out = controller->out_min;
			//	}

	//updated the error and measured position
			//	controller->error = updated_error;
			//	controller->measured_pos = updated_measured_value;

}
void update_motor_input(int16_t new_out, uint32_t **active_buffer_address, uint32_t **inactive_buffer_address)
{
//	if (new_out > 0)
//	{
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
//
//	}
//	else if (new_out < 0)
//	{
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
//		new_out = new_out * -1;
//	}

	**inactive_buffer_address = new_out;
	uint32_t *temp_uint32_address = *active_buffer_address;
	*active_buffer_address = *inactive_buffer_address;
	*inactive_buffer_address = temp_uint32_address;

}
