/*#include "throttle_control.h"
#include "math.h"
#define PI 3.142857

int Analog_to_throttle(int analog)
{
	float throttle = 0;
	if (analog < 1)
		throttle = 0.0;
	else if (analog > 4095)
		throttle = 100.0;
	else
	{
		throttle = (analog / 4095) * 100;
	}
	throttle = round(throttle);
	return throttle;
}

int Throttle_to_analog(int throttle)
{
	float analog = 0;
	if (throttle >= 100)
		analog = 4095;
	else if (throttle <= 0)
		analog = 0;
	else
	{
		analog = (throttle * 4095) / 100;
	}
	analog = round(analog);
	return analog;
}

int Trac_Control_Run(Trac_Control *control)
{
	control->running = 0;
	int temp_throttle = control->throttle_percent;
	float difference = Slippage(control->Front_Speed, control->Rear_Speed);
	if (difference > control->Slippage_Target)
	{
		control->error = control->Slippage_Target - difference;
		control->throttle_percent = control->throttle_percent + porportional(control->error) + integral(control->error, control->time) + derivative(control->error, control->prev_error, control->time);
		++control->time;
		control->prev_error = control->error;
		control->running = 1;
	}
	else
	{
		Reset_Pid(control);
	}
	if (temp_throttle < control->throttle_percent)
		return temp_throttle;
	return control->throttle_percent;
}

void Reset_Pid(Trac_Control *control)
{
	control->time = 0;
	control->prev_error = 0.0;
}

void Trac_Control_Init(Trac_Control *control)
{
	Por = 187;
	In = 6;
	Der = 144;
	control->Slippage_Target = 1.07;
	control->max_throttle = 100;
}

void Slip_Target_Change(Trac_Control *control, int value)
{
	control->off = 0;
	if (value == 0)
		control->Slippage_Target = 1.07;
	if (value == 1)
			control->Slippage_Target = 1.08;
	if (value == 2)
			control->Slippage_Target = 1.09;
	if (value == 3)
			control->off = 1;
}

void Max_Throttle_Change(Trac_Control *control, int value)
{
	if (value == 0)
				control->max_throttle = 50;
	if (value == 1)
				control->max_throttle = 65;
	if (value == 2)
				control->max_throttle = 80;
	if (value == 3)
				control->max_throttle = 100;


}

unsigned long Frequency_to_MPH(unsigned long frequency)
{
	float mph = 0.0;
	mph = (float) frequency;
	mph = (mph / 200) * 60;
	mph = mph * 18;
	mph = mph * (60 * PI);
	mph = mph / 63360;
	mph = round(mph);
	return mph;
}

unsigned long Front_Wheel_Average(unsigned long FR_Frequency, unsigned long FL_Frequency) //FIX LATER
{
	return ((FR_Frequency + FL_Frequency) / 2);
}

float Slippage(long Front_Speed, long Rear_Speed)
{
	float slippage = (float) Rear_Speed / (float) Front_Speed;
	return slippage;
}

float porportional(float error)
{
    float P = Por * error;
    return P;
}

float integral(float error, float time)
{
    float I = In * error * time;
    return I;
}

float derivative(float error, float prevError, float t)
{
    if (t == 0)
        return 0;
    float D = Der  * (error * prevError) / t;
    return D;
}*/
