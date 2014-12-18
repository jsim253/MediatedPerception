/* Copyright (C) 2014 Digipen Institute of Technology */
#include "LowPassFilter.h"


LowPassFilter::LowPassFilter(int sampleSize): counter(0) // constructor
{
	if(sampleSize < 0)
	{
		sampleSize = 0;
	}
	_data = new float[sampleSize-1];
}

LowPassFilter::~LowPassFilter() // destructor
{
	delete [] _data;
}

bool LowPassFilter::ApplyFilter(float data, float & XFilter)
{
	if(counter == 0)
	{
		_data[0] = data;
		++counter;
		return false;
	}
	else if(counter == 1)
	{
		_data[1] = data;
		++counter;
		return false;
	}
	else if(counter == 2)
	{
		++counter;
		return false;
	}
	else if(counter == 3)
	{
		_data[2] = data;
		++counter;
		return false;
	}
	else if(counter == 4)
	{
		_data[3] = data;
		++counter;				
		return false;
	}
	else if(counter == 5)
	{
		//Filter
		XFilter = (1.0f/5.0f)*(_data[0] + _data[1] + data + _data[2] + _data[3]);
		counter = 0;
		return true;
	}
	return true;
}
