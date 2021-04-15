#include "ch.h"
#include "hal.h"
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <control.h>
#include <arm_math.h>
#include <arm_const_structs.h>

//2 times FFT_SIZE because this array contains complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
//Array containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

//sound remote paramaters
#define MIN_VALUE_THRESHOLD 10000
#define MIN_FREQ			100	//we don't analyze before this index to not use resources for nothing
#define FREQ_FORWARD		130	//2000Hz
#define FREQ_LEFT			190	//3000Hz
#define FREQ_RIGHT			262	//4000Hz
#define FREQ_BACKWARD		327	//5000Hz
#define MAX_FREQ			350	//we don't analyze after this index to not use resources for nothing

#define FREQ_FORWARD_L		(FREQ_FORWARD-10)
#define FREQ_FORWARD_H		(FREQ_FORWARD+10)
#define FREQ_LEFT_L			(FREQ_LEFT-10)
#define FREQ_LEFT_H			(FREQ_LEFT+10)
#define FREQ_RIGHT_L		(FREQ_RIGHT-10)
#define FREQ_RIGHT_H		(FREQ_RIGHT+10)
#define FREQ_BACKWARD_L		(FREQ_BACKWARD-10)
#define FREQ_BACKWARD_H		(FREQ_BACKWARD+10)


void sound_remote(float *data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	for (uint16_t i = MIN_FREQ; i < MAX_FREQ; i++)
	{
		if (data[i] > max_norm)
		{
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	//go forward
	if(max_norm_index >= FREQ_FORWARD_L && max_norm_index <= FREQ_FORWARD_H)
	{
		skip_stop();
		skip_stop();
		clr_intersect();
	}
	//turn left
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H)
	{
		turn_signal(LEFT);
		skip_stop();
		skip_stop();
		rotate_left();
		clr_intersect();
	}
	//turn right
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H)
	{
		turn_signal(RIGHT);
		skip_stop();
		skip_stop();
		rotate_right();
		clr_intersect();
	}
	//u turn
	else if(max_norm_index >= FREQ_BACKWARD_L && max_norm_index <= FREQ_BACKWARD_H)
	{
		turn_signal(UTURN);
		u_turn();
		clr_intersect();
	}

	else
	{
		stay_stop();
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples)
{

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4)
	{
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		nb_samples++;
		micLeft_cmplx_input[nb_samples] = 0;
		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE))
			break;
	}

	if(nb_samples >= (2 * FFT_SIZE))
	{
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		nb_samples = 0;

		if (get_intersect())
			sound_remote(micLeft_output);
	}
}


float* get_audio_buffer_ptr(BUFFER_NAME_t name)
{
	if(name == LEFT_CMPLX_INPUT)
		return micLeft_cmplx_input;

	else if (name == LEFT_OUTPUT)
		return micLeft_output;

	else
		return NULL;
}

/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of tricks to optimize the computations
*/
void doFFT_optimized(uint16_t size, float* complex_buffer)
{
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);

}
