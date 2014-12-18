/* Copyright (C) 2014 Digipen Institute of Technology */
#include "Kalman.h"

float Kalman::KalmanCalculate(float newAngle, float newRate, float looptime) 
{
	m_dt = looptime;                                  
	m_x_angle += m_dt * (newRate - m_x_bias);
	m_P_00 +=  - m_dt * (m_P_10 + m_P_01) + m_Q_angle * m_dt;
	m_P_01 +=  - m_dt * m_P_11;
	m_P_10 +=  - m_dt * m_P_11;
	m_P_11 +=  + m_Q_gyro * m_dt;
    
	m_y = newAngle - m_x_angle;
	m_S = m_P_00 + m_R_angle;
	m_K_0 = m_P_00 / m_S;
	m_K_1 = m_P_10 / m_S;
    
	m_x_angle +=  m_K_0 * m_y;
	m_x_bias  +=  m_K_1 * m_y;
	m_P_00 -= m_K_0 * m_P_00;
	m_P_01 -= m_K_0 * m_P_01;
	m_P_10 -= m_K_1 * m_P_00;
	m_P_11 -= m_K_1 * m_P_01;
    
	return m_x_angle;
}

Kalman::Kalman()
{
	m_x_angle = 0;
	m_x_bias = 0;
	m_P_00 = 0;
	m_P_01 = 0;
	m_P_10 = 0;
	m_P_11 = 0;   
}
		
void Kalman::SetCalibration(float Q_angle, float Q_gyro, float R_angle)
{
	m_Q_angle = Q_angle;
	m_Q_gyro   =  Q_gyro;
	m_R_angle  =  R_angle; 

}
