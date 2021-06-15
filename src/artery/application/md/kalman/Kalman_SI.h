/*******************************************************************************
 * @author  Joseph Kamel
 * @email   josephekamel@gmail.com
 * @date    28/11/2018
 * @version 2.0
 *
 * SCA (Secure Cooperative Autonomous systems)
 * Copyright (c) 2013, 2018 Institut de Recherche Technologique SystemX
 * All rights reserved.
 *******************************************************************************/
#ifndef __VEINS_Kalman_SI_SI
#define __VEINS_Kalman_SI_SI
#include <cmath>
#include <iostream>
#include "KalmanFilterJ_SI.h"
using namespace std;

class Kalman_SI {

private:	
	void setT(double T);
	void setConfidence(double CX, double CY);
	double A[KLM_N_SI][KLM_N_SI];
	double B[KLM_N_SI][KLM_M_SI];
	double R[KLM_N_SI][KLM_N_SI];
	double P0[KLM_N_SI][KLM_N_SI]; 
	double X0[KLM_N_SI];

	double U[KLM_N_SI];

	bool init = false;
	
public:	
	KalmanFilterJ_SI kalmanFilterJ_SI;
	Kalman_SI();
	bool isInit() const;
	void setInitial(double X, double Y);
	void getDeltaPos(double T, double X, double Y, double CX, double CY, double * Delta);
	void getDeltaPos(double T, double X, double Y, double AX, double AY, double CX, double CY, double * Delta);

};
#endif




