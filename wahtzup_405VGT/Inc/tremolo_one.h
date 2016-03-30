/*
 *by scott monk
 *march 28 2016
 */

#ifndef __TREMOLO_ONE_H
#define __TREMOLO_ONE_H

struct tremolo_one_lfo{
	int32_t is_active;
	float t; 	//index
	float T;	//period
};

int32_t tremolo_one( float * input_buffer, float * output_buffer, int32_t buffer_size);

#endif