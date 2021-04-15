#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <selector.h>
#include <control.h>
#include <leds.h>

//robot speed can be selected by the user
#define SPEED0      200
#define SPEED1      250
#define SPEED2		300
#define SPEED3		350
#define SPEED4		375
#define SPEED5		400
#define SPEED6		425
#define SPEED7		450

//the robot can only be in two states :
//STOP and wait for frequency peak or FOLLOW line
//initially the robot is in STOP state
static uint8_t intersect = 1;

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal)
{

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is too small
	if(fabs(error) < ERROR_THRESHOLD)
		return 0;

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR)
		sum_error = MAX_SUM_ERROR;
	else if(sum_error < -MAX_SUM_ERROR)
		sum_error = -MAX_SUM_ERROR;

	speed = KP * error + KI * sum_error;

	//robot is only meant to slow down or stop if something is in the way
	if (speed < 0)
		speed = 0;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waSpeedRegulator, 256);
static THD_FUNCTION(SpeedRegulator, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1)
    {
        time = chVTGetSystemTime();
        static uint16_t previous_pos = IMAGE_BUFFER_SIZE/2;
        
        //distance_mm is modified by the TOF thread
        //we convert dist_mm to cm
        float dist_cm = VL53L0X_get_dist_mm() / (float)10;

        //we get the line width in order to detect a stop
        uint16_t width = get_width();


        //INTERSECTION DETECTION
        if (width < MAX_WIDTH && width > MIN_WIDTH)
        {
        	uint8_t i = 0;
        	while (width > MIN_WIDTH && width < MAX_WIDTH)
        	{
        		i++;
        		width = get_width();

        		//if we can detect MIN_STOP_NB times the width for an intersection
        		//and if we are not in a turn, we can set intersect
        		if (i> MIN_STOP_NB && speed_correction < 2*ROTATION_THRESHOLD)
        		{
					speed = 0;
					speed_correction = 0;
					i = 0;
					width = 0;
					set_intersect();
					chThdSleepMilliseconds(1000);//thread sleep to insure stop is long enough
        		}
        		chThdSleepMilliseconds(10);// thread sleep to get new width
        	}
        }


        //LINE FOLLOWING
        //computes the speed to give to the motors
        else if (!intersect)
        {
			if (dist_cm < (MIN_DISTANCE - DIST_GAP) )
			{
				speed = pi_regulator(dist_cm, GOAL_DISTANCE);
			}
			else if (dist_cm >= (GOAL_DISTANCE + DIST_GAP) )
				speed = get_speed();

			//computes a correction factor to let the robot rotate to be in front of the line
			if (speed)
			{
				speed_correction = ((get_line_position()*NOW_COEFF+ previous_pos*PREVIOUS_COEFF)/100 \
				- (IMAGE_BUFFER_SIZE/2));//moving average to smooth rotation

			}
			else
				speed_correction = 0;

			previous_pos = get_line_position();

			//if the line position is nearly in the center of the camera, don't rotate
			if(abs(speed_correction) < ROTATION_THRESHOLD)
			{
				speed_correction = 0;
			}
		}

        else
        {
        	speed = 0;
        	speed_correction = 0;
        }

		//applies the speed and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void speed_regulator_start(void)
{
	chThdCreateStatic(waSpeedRegulator, sizeof(waSpeedRegulator), NORMALPRIO, SpeedRegulator, NULL);
}

//VARIOUS MOTOR CONTROL FUNCTIONS
//-----------------------------------------------------------------------------------------
void rotate_right(void)
{
	for(uint32_t i = 0; i<ROTATE_90; i++)
	{
		left_motor_set_speed(+CONTROL_SPEED);
		right_motor_set_speed(-CONTROL_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void rotate_left(void)
{
	for(uint32_t i = 0; i<ROTATE_90; i++)
	{
		left_motor_set_speed(-CONTROL_SPEED);
		right_motor_set_speed(+CONTROL_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void skip_stop(void)
{
	for(uint32_t i = 0; i<SKIP_STEPS; i++)
	{
		left_motor_set_speed(+CONTROL_SPEED);
		right_motor_set_speed(+CONTROL_SPEED);
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void stay_stop(void)
{
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void u_turn(void)
{
	rotate_left();
	rotate_left();
}

//-----------------------------------------------------------------------------------------

void turn_signal(uint8_t side)
{
	for (uint8_t i=0; i<BLINK_NB; i++)
	{
		if (side == LEFT)
			set_led(LED7, 1);
		else if (side == RIGHT)
			set_led(LED3, 1);
		else if (side == UTURN)
		{
			set_led(LED3, 1);
			set_led(LED7, 1);
		}

		chThdSleepMilliseconds(300);
		clear_leds();
		chThdSleepMilliseconds(300);
	}
}

//STATE VARIABLE FUNCTIONS
//-----------------------------------------------------------------------------------------
void set_intersect(void)
{
	intersect = 1;
}
void clr_intersect(void)
{
	intersect = 0;
}

uint8_t get_intersect(void)
{
	return intersect;
}
//-----------------------------------------------------------------------------------------

//speed from selector
uint16_t get_speed(void)
{
	switch(get_selector())
	{
		case 0:
			return SPEED0;
		case 1:
			return SPEED0;
		case 2:
			return SPEED1;
		case 3:
			return SPEED1;
		case 4:
			return SPEED2;
		case 5:
			return SPEED2;
		case 6:
			return SPEED3;
		case 7:
			return SPEED3;
		case 8:
			return SPEED4;
		case 9:
			return SPEED4;
		case 10:
			return SPEED5;
		case 11:
			return SPEED5;
		case 12:
			return SPEED6;
		case 13:
			return SPEED6;
		case 14:
			return SPEED7;
		case 15:
			return SPEED7;
		default:
			return SPEED0;
	}
}


