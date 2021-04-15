#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <control.h>
#include <process_image.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <leds.h>

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //body led ON
    set_body_led(1);

    //starts the camera
    dcmi_start();
	po8030_start();

	//inits the motors
	motors_init();

	//starts the threads for the speed regulator and the processing of the image
	speed_regulator_start();
	process_image_start();

	//starts the TOF thread
	VL53L0X_start();

	//starts audio processing from left mic
	mic_start(&processAudioData);

    /* Infinite loop. */
    while (1)
    {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
