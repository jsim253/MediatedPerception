#include "mbed.h"
#include "math.h"
#include "../MPU6050/MPU6050.h"
#include "../MPU6050/Kalman.h"
#include "../HMC5883L/HMC5883L.h"
 
DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);

namespace Peripherals
{
	MPU6050 mpu;
	HMC5883L comp;
}

int main()
{	
	/* Debugging Values */
	float x, y, z;
	float gx, gy, gz;
	
	/* Showing it got into main */
	myled = 1;
	
	/* Initializing Accelerometer and Gyroscope */
	/* - Sends the appropriate commands to the IC2 Line */
	Peripherals::mpu.InitializeEverything();
	
	/* Make sure I2C Object are assigned */
	Peripherals::comp.AssignI2CObject(Peripherals::mpu.getI2C());
	/* Set Up Compass*/
	Peripherals::comp.SetupHMC5883LDevice();
	
	/* Loop To Initialize to Fetch Data and Apply Corresponding Filters */
	while(1)
	{
		/* Sample Filter */
		Peripherals::comp.FetchData();
		
		Peripherals::comp.FillAxis(x, y, z);
		
		/* Accelerometer and Gyro Update */
		Peripherals::mpu.Update();
		Peripherals::mpu.ReturnGyroscope(gx,gy,gz);
		float Pitch = Peripherals::mpu.GetPitch();
		float Roll = Peripherals::mpu.GetRoll();
		/* Compass Update and Magnetic North Fetching */
		float MagneticNorth = Peripherals::comp.MagneticNorthCalculations(Pitch, Roll);
		//float Yaw = Peripherals::mpu.GetYaw(MagneticNorth);
		
		
		//pc.printf("Pitch: %f; Roll: %f; Yaw: %f; Cx: %f; Cy: %f, Cz: %f; \n\r", Pitch, Roll, MagneticNorth, x,y,z);
		pc.printf("Pitch: %f; Roll: %f; Yaw: %f;\n\r",Pitch, Roll, MagneticNorth);
	}
}
