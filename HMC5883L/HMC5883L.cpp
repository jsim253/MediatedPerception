/* Copyright (C) 2014 Digipen Institute of Technology */
#include "HMC5883L.h"

#define X_SF 1.09195f
#define Y_SF 1
#define X_OFF -7.097f
#define Y_OFF -30.5f

void HMC5883L::SetupHMC5883LDevice()
{
		I2CObject->frequency(Standard);
		//setup continuous mode
		int buffer[1];
		buffer[0] = HMC5883_L_75HZ_NORMAL;
		Write(ConfigA, buffer,1);
		buffer[0] = HMC5883_L_E0GA;
		Write(ConfigB, buffer,1);
		buffer[0] = HMC5883_L_CONTINUOUS;
		Write(ModeSelect ,buffer, 1); 
		
		timer.start();
		tx.start();
		ty.start();
		tz.start();
	
		m_x.SetCalibration(1, .0003, .1);
		m_y.SetCalibration(1, .0003, .1);
		m_z.SetCalibration(1, .3, .1);
}

void HMC5883L::Write(int dataOutputReg, int * buffer, int length)
{
	int ack = 0;
	char transmit[length+1];
	transmit[0] = dataOutputReg;
	for(int i = 1; i <= length; ++i)
	{
		transmit[i] = buffer[i-1];
	}
	I2CObject->write(writeAddress, transmit, length+1);
	if(ack != 0)
	{
		return;
	}
}

bool HMC5883L::FetchData()
{
	int16_t xM,yM,zM;
	
	int CompassReadings[3];
	SampleCompass(CompassReadings);
	
	xM = -(int16_t)CompassReadings[0];
	yM = (int16_t)CompassReadings[2];
	zM = (int16_t)CompassReadings[1];
	
	
	xM = m_x.KalmanCalculate(xM, txM, tx.read());
	yM = m_y.KalmanCalculate(yM, tyM, ty.read());
	zM = m_z.KalmanCalculate(zM, tzM, tz.read());
	
	txM = xM;
	tyM = yM;
	tzM = zM;
	
	tx.reset();
	ty.reset();
	tz.reset();

	XFilter = xM;
	YFilter = yM;
	ZFilter = zM;
	
	return true;
	/*
	bool result1 = xAxis.ApplyFilter(xM,XFilter);
	bool result2 = yAxis.ApplyFilter(yM,YFilter);
	bool result3 = zAxis.ApplyFilter(zM,ZFilter);
	
	if(result1 == false || result2 == false || result3 == false)
	{
		return false;
	}
	return true;
	*/
}


float HMC5883L::MagneticNorthCalculations(float Pitch, float Roll)
{
	int16_t temp = (X_SF*XFilter) + X_OFF;
	int16_t temp2= (Y_SF*YFilter) + Y_OFF;
	int16_t temp3= ZFilter;
	
	int16_t newX = (temp*cos((Pitch*PI)/180)) + (temp2*sin((Roll*PI)/180)*sin(Pitch*PI/180)) - (temp3*cos((Roll*PI)/180)*sin((Pitch*PI)/180));
	int16_t newY = (temp2*cos((Roll*PI)/180)) + (temp3*sin((Roll*PI)/180));
				
	filter.SetCalibration(1.0f, .03f, .1f);
	float north = Azimuth(newX, newY);
	//return north;
//	float newVal = filter.KalmanCalculate(north, MagneticNorthO, timer.read());
//	MagneticNorthO = newVal;
//	return newVal; // Angular Offset
	return north;
}



void HMC5883L::FillAxis(float & x, float & y, float & z)
{
	x = XFilter;
	y = YFilter;
	z = ZFilter;
}


float HMC5883L::Azimuth(int16_t & x, int16_t & y)
{
	if(x==0 && y < 0) 
		return 90.0f;
	if(x==0 && y > 0)
		return 270.0f;
	if(x < 0)
		return (180.0f - ((atan((float)y/(float)x))*180.0f/PI));
	if(x > 0 && y < 0)
		return -((atan((float)y/(float)x))*180.0f/PI);
	if(x > 0 && y > 0)
		return (360.0f - (atan((float)y/(float)x)*180.0f/PI));
	return 0.0f;
}



void HMC5883L::AssignI2CObject(I2C & Object)
{
	I2CObject = &Object;
}

void HMC5883L::NewDataReady()
{
	char tx[1];
	char rx[1];
	int ready = 0;
	int count = 0;
	while(ready == 0)
	{
		tx[0] = statusRegister;
		I2CObject->write(readAddress, tx, 1);
		I2CObject->read(readAddress, rx, 1);
		ready = (int)rx[0];
		if(ready != 1)
		{
			ready = 0;
		}
		++count;
	}
	count= 0;
}

void HMC5883L::SampleCompass(int * data )
{
	char tx[1];
	char rx[6];
	
	tx[0]=HMC5883_L_X_MSB;
	I2CObject->write(readAddress,tx,1);
	I2CObject->read(readAddress,rx,6);
	data[0]= (int)rx[0]<<8|(int)rx[1];
	data[0] = ~data[0] + 1;
	data[1]= (int)rx[2]<<8|(int)rx[3];
	data[1] = ~data[1] + 1;
	data[2]= (int)rx[4]<<8|(int)rx[5];
	data[2] = ~data[2] + 1;
	
		//Check Status Register before reading 
	//NewDataReady();

}
