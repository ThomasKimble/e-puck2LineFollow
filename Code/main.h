#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the different parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			12.0f
#define MIN_DISTANCE			6.0f
#define MAX_DISTANCE 			25.0f
#define DIST_GAP				1
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						500.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define BLINK_NB				3 		//for turn signal
#define LINE_NB					350		//camera line to be processed
#define CONTROL_SPEED			300		//speed for motor control functions
#define ROTATE_90				75000	//clock ticks for 90 degrees rotation
#define SKIP_STEPS				40000	//clock ticks for 2cm stop skip
#define MIN_STOP_NB				10		//for intersection detection

//in order to smooth the line following a moving average
//is performed on the line position using these coefficients:
//total must be 100
#define PREVIOUS_COEFF	30
#define	NOW_COEFF		70

//turn signal parameter
#define UTURN		2
#define LEFT		1
#define RIGHT		0

//width for intersection detection
#define MIN_WIDTH	300
#define MAX_WIDTH	450

/** Robot wide IPC bus. */
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
