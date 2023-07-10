#include <stdbool.h>

unsigned int Por;
unsigned int In;
unsigned int Der;

typedef struct Trac_Control {
	float Slippage_Target; //Optimal Slippage target
	unsigned long Front_Speed; //Front wheel speed in MPH
	unsigned long Rear_Speed; //Rear wheel speed in MPH
	unsigned int max_throttle;
	unsigned int time;
	unsigned int throttle_percent;
	float prev_error;
	float error;
	bool off;
	bool running;
} Trac_Control;


void Slip_Target_Change(Trac_Control *control, int value);
void Max_Throttle_Change(Trac_Control *control, int value);
int Analog_to_throttle(int analog);
int Throttle_to_analog(int throttle);
int Trac_Control_Run(Trac_Control *control);
void Trac_Control_Init(Trac_Control *control);
unsigned long Frequency_to_MPH(unsigned long frequency);
unsigned long Front_Wheel_Average(unsigned long FR_Frequency, unsigned long FL_Frequency);
float Slippage(long Front_Speed, long Rear_Speed);
void Reset_Pid(Trac_Control *control);
float porportional(float error);
float integral(float error, float time);
float derivative(float error, float prevError, float time);
