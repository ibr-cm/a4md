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
#ifndef __VEINS_Kalman_SVI_SVI
#define __VEINS_Kalman_SVI_SVI

#include <cmath>
#include <iostream>
#include "KalmanFilterJ_SVI.h"

using namespace std;

class Kalman_SVI {

private:
    void setT(double T);

    void setConfidence(double CX, double CY, double CV, double CH);

    double A[KLM_N_SVI][KLM_N_SVI];
    double B[KLM_N_SVI][KLM_M_SVI];
    double R[KLM_N_SVI][KLM_N_SVI];
    double P0[KLM_N_SVI][KLM_N_SVI];
    double X0[KLM_N_SVI];

    double U[KLM_N_SVI];

    bool init = false;

public:
    KalmanFilterJ_SVI kalmanFilterJ_SVI;

    Kalman_SVI();

    bool isInitialized() const;

    void setInitial(double X, double Y, double V, double H);

    void getDeltaPos(double T, double X, double Y, double V, double H, double CX, double CY, double CV, double CH,
                     double *Delta);

    void
    getDeltaPos(double T, double X, double Y, double V, double H, double AX, double AY, double CX, double CY, double CV,
                double CH, double *Delta);

};

#endif




