/*tremelo_one.c
 *	a basic tremolo
 *	by scott monk
 *	march 28, 2016
 */
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_def.h"
#include "system_defs.h"
#include "tremolo_one.h"



static struct tremolo_one_lfo tremolo_lfo = {0,0,0};

int32_t  tremolo_one( float * input_buffer, float  * output_buffer, int32_t buffer_length){

	//init the lfo
	static struct tremolo_one_lfo  * my_lfo =  & tremolo_lfo;
	if(my_lfo->is_active != 1){
		my_lfo->T = 5;
		my_lfo->t = 0;
		my_lfo->is_active = 1;
	}


	float t_increment = 1.0/SAMPLING_RATE;
	float amplitude_left, amplitude_right  = 0;
	int32_t l, r;
	for( l = 0; l < buffer_length; l+=2){

		//left and right index
		r = l + 1;

		//increment the lfo
		my_lfo->t += t_increment;

		//check if time has expired
		if(my_lfo->t >= my_lfo->T){
			my_lfo->t -= my_lfo->T;
		}

		//now convert the lfo postion to an amplitude via some function. here is a boring one
		amplitude_left = my_lfo->t/my_lfo->T;
		amplitude_right = 1.0 - amplitude_left;

		//now scale the data
		output_buffer[l] = input_buffer[l] * amplitude_left;
		output_buffer[r] = input_buffer[r] * amplitude_right;


	}


	return 0;


}
