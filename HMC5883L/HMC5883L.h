/* Copyright (C) 2014 Digipen Institute of Technology */
#ifndef HMC5883L_HEADER
#define HMC5883L_HEADER

#include "mbed.h"
#include "../MPU6050/Kalman.h"

#define SAMPLE 2
#define PI 3.14

#define HMC5883_L_X_MSB        0x03
#define HMC5883_L_X_LSB        0x04
#define HMC5883_L_Y_MSB        0x05
#define HMC5883_L_Y_LSB        0x06
#define HMC5883_L_Z_MSB        0x07
#define HMC5883_L_Z_LSB        0x08
#define HMC5883_L_15HZ_NORMAL  0x10
#define HMC5883_L_75HZ_NORMAL  0x78
#define HMC5883_L_1_0GA        0x20
#define HMC5883_L_0_0GA				 0x00
#define HMC5883_L_E0GA				 0xE0
#define HMC5883_L_CONTINUOUS   0x00
#define HMC5883_L_SINGLE			 0x01
#define LOCK_REGISTER 				 0x11
#define LOCK_HOLD							 0x10

const int writeAddress = 0x3C; 
const int readAddress = 0x3D;
const int statusRegister = 0x09;
const int ConfigA = 0x00; // Read/Write
const int ConfigB = 0x01; // Read/Write
const int ModeSelect = 0x02;

const int Standard = 100000; //Standard Operation
const int Fast = 400000; // Fast Operation

class HMC5883L
{
	public:
		HMC5883L(): XFilter(0.0f), YFilter(0.0f), ZFilter(0.0f), MagneticNorthN(0.0f),
								MagneticNorthO(0.0f) {}
		~HMC5883L() {}
		void AssignI2CObject(I2C & Object);
		void SetupHMC5883LDevice();
		void Read(int dataOutputReg, int * buffer, int length);
		void Write(int dataOutputReg, int * buffer, int length);
		void SampleCompass(int * data );
		void NewDataReady();
		
		
		float MagneticNorthCalculations(float Pitch, float Roll);	
		bool FetchData();
		float Azimuth(int16_t & x, int16_t & y);
			
			
		/* Debugging Purposes */
		void FillAxis(float & x, float & y, float & z);
			
	private:
		I2C * I2CObject;
		int16_t XFilter, YFilter, ZFilter;
		Timer timer;
		Timer tx;
		Timer ty;
		Timer tz;
		Kalman m_x,m_y,m_z;
		int16_t txM,tyM,tzM;
		Kalman filter;
		float MagneticNorthN;
		float MagneticNorthO;
};

#endif
