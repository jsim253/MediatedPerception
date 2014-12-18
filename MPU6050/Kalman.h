/* Copyright (C) 2014 Digipen Institute of Technology */
#ifndef KALMAN
#define KALMAN

#include "Kalman.h"

class Kalman
{
	public:
		Kalman();
		void SetCalibration(float Q_angle, float Q_gyro, float R_angle);
		float KalmanCalculate(float newAngle, float newRate,float looptime);
		
	private:
		float m_x_angle;
		float m_x_bias;
		float m_P_00, m_P_01, m_P_10, m_P_11;      
		float m_dt, m_y, m_S;
		float m_K_0, m_K_1;
	
		float m_Q_angle;
		float m_Q_gyro;
		float m_R_angle; 
};

#endif



