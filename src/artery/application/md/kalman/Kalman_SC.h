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
#ifndef __VEINS_Kalman_SC_SC
#define __VEINS_Kalman_SC_SC

#include <cmath>
#include <iostream>
#include "KalmanFilterJ_SC.h"

using namespace std;

class Kalman_SC {

private:
    void setT(double T);

    void setConfidence(double CX, double CY);

    double pos = 0;
    double A[KLM_N_SC][KLM_N_SC];
    double B[KLM_N_SC][KLM_M_SC];
    double R[KLM_N_SC][KLM_N_SC];
    double P0[KLM_N_SC][KLM_N_SC];
    double X0[KLM_N_SC];

    double U[KLM_N_SC];

    bool init = false;

public:
    KalmanFilterJ_SC kalmanFilterJ_SC;

    Kalman_SC();

    bool isInitialized() const;

    void setInitial(double X, double Y);

    void getDeltaPos(double T, double X, double Y, double CX, double CY, double *Delta);

    void getDeltaPos(double T, double X, double Y, double AX, double AY, double CX, double CY, double *Delta);

};

#endif




