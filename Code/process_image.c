#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <main.h>
#include <camera/po8030.h>
#include <process_image.h>
#include <control.h>

static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint16_t width = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Computes the line's width extracted from the image buffer given
 *  width is 0 if line not found
 */
void extract_line_width(uint8_t *buffer)
{
	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++)
	{
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do
	{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH)
		{
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}
	while(wrong_line);

	if(line_not_found)
	{
		begin = 0;
		end = 0;
		width = last_width;
		line_position = IMAGE_BUFFER_SIZE/2;

	}
	else
	{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width
	if((PXTOCM/width) > MAX_DISTANCE)
		width = PXTOCM/MAX_DISTANCE;

}

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 350 + 351 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, LINE_NB, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1)
    {
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(1)
    {
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//extract green pixels instead of red for more precision
		for (int i = 0; i < 2*IMAGE_BUFFER_SIZE; i += 2)
		{
			image[i/2] = ((img_buff_ptr[i] & 7) << 3) | ((img_buff_ptr[i+1] & 0xE0) >> 5);
		}

		extract_line_width(image);
    }
}

uint16_t get_line_position(void)
{
	return line_position;
}

void process_image_start(void)
{
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}


uint16_t get_width (void)
{
	return width;
}
