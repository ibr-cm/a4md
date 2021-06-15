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

#include "Kalman_SI.h"
using namespace std;

Kalman_SI::Kalman_SI() {

  double T = 1;

  A[0][0] = 1;
  A[0][1] = 0;

  A[1][0] = 0;
  A[1][1] = 1;

  B[0][0] = T;
  B[1][0] = T;

  double H[KLM_N_SI][KLM_N_SI];

  H[0][0] = 1;
  H[0][1] = 0;

  H[1][0] = 0;
  H[1][1] = 1;

  double Q[KLM_N_SI][KLM_N_SI];
  Q[0][0] = 10;
  Q[0][1] = 0;

  Q[1][0] = 0;
  Q[1][1] = 10;

  R[0][0] = 5;
  R[0][1] = 0;

  R[1][0] = 0;
  R[1][1] = 5;

  kalmanFilterJ_SI.setFixed(A, H, Q, R, B);

  init = false;

}

bool Kalman_SI::isInit() const{
  return init;
}


void Kalman_SI::setInitial(double X, double Y){

  P0[0][0] = 10;
  P0[0][1] = 0;

  P0[1][0] = 0;
  P0[1][1] = 10;

  X0[0] = X;
  X0[1] = Y;

  kalmanFilterJ_SI.setInitial(X0, P0);

  init = true;
}

void Kalman_SI::setT(double T){

  A[0][0] = 1;
  A[0][1] = 0;

  A[1][0] = 0;
  A[1][1] = 1;

  B[0][0] = T;
  B[1][0] = T;

  kalmanFilterJ_SI.setA(A);
  kalmanFilterJ_SI.setB(B);
}

void Kalman_SI::setConfidence(double CX, double CY){

  R[0][0] = CX;
  R[0][1] = 0;


  R[1][0] = 0;
  R[1][1] = CY;

  kalmanFilterJ_SI.setR(R);
}

void Kalman_SI::getDeltaPos(double T, double X, double Y, double CX, double CY, double * Delta){

    setT(T);
    setConfidence(CX,CY);

    X0[0] = X;
    X0[1] = Y;

    kalmanFilterJ_SI.predict();
    kalmanFilterJ_SI.correct(X0);

    Delta[0] = fabs(kalmanFilterJ_SI.X[0] - X0[0]);
    Delta[1] = fabs(kalmanFilterJ_SI.X[1] - X0[1]);

}


void Kalman_SI::getDeltaPos(double T, double X, double Y, double AX, double AY, double CX, double CY, double * Delta){

    setT(T);
    setConfidence(CX,CY);

    X0[0] = X;
    X0[1] = Y;

    U[0] = AX;
    U[1] = AY;

    kalmanFilterJ_SI.predict(U);
    kalmanFilterJ_SI.correct(X0);

    Delta[0] = fabs(kalmanFilterJ_SI.X[0] - X0[0]);
    Delta[1] = fabs(kalmanFilterJ_SI.X[1] - X0[1]);

}


