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

#include "Kalman_SVI.h"

using namespace std;

Kalman_SVI::Kalman_SVI() {

    double T = 1;

    A[0][0] = 1;
    A[0][1] = T;
    A[0][2] = 0;
    A[0][3] = 0;

    A[1][0] = 0;
    A[1][1] = 1;
    A[1][2] = 0;
    A[1][3] = 0;

    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = 1;
    A[2][3] = T;

    A[3][0] = 0;
    A[3][1] = 0;
    A[3][2] = 0;
    A[3][3] = 1;

    B[0][0] = pow(T, 2.0f) / 2.0f;
    B[1][0] = T;
    B[2][0] = pow(T, 2.0f) / 2.0f;
    B[3][0] = T;

    double H[KLM_N_SVI][KLM_N_SVI];

    H[0][0] = 1;
    H[0][1] = 0;
    H[0][2] = 0;
    H[0][3] = 0;

    H[1][0] = 0;
    H[1][1] = 1;
    H[1][2] = 0;
    H[1][3] = 0;

    H[2][0] = 0;
    H[2][1] = 0;
    H[2][2] = 1;
    H[2][3] = 0;

    H[3][0] = 0;
    H[3][1] = 0;
    H[3][2] = 0;
    H[3][3] = 1;


    double Q[KLM_N_SVI][KLM_N_SVI];
    Q[0][0] = 50;
    Q[0][1] = 0;
    Q[0][2] = 0;
    Q[0][3] = 0;

    Q[1][0] = 0;
    Q[1][1] = 10;
    Q[1][2] = 0;
    Q[1][3] = 0;

    Q[2][0] = 0;
    Q[2][1] = 0;
    Q[2][2] = 50;
    Q[2][3] = 0;

    Q[3][0] = 0;
    Q[3][1] = 0;
    Q[3][2] = 0;
    Q[3][3] = 10;

    R[0][0] = 5;
    R[0][1] = 0;
    R[0][2] = 0;
    R[0][3] = 0;

    R[1][0] = 0;
    R[1][1] = 1;
    R[1][2] = 0;
    R[1][3] = 0;

    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = 5;
    R[2][3] = 0;

    R[3][0] = 0;
    R[3][1] = 0;
    R[3][2] = 0;
    R[3][3] = 1;


    P0[0][0] = 0;
    P0[0][1] = 0;
    P0[0][2] = 0;
    P0[0][3] = 0;

    P0[1][0] = 0;
    P0[1][1] = 0;
    P0[1][2] = 0;
    P0[1][3] = 0;

    P0[2][0] = 0;
    P0[2][1] = 0;
    P0[2][2] = 0;
    P0[2][3] = 0;

    P0[3][0] = 0;
    P0[3][1] = 0;
    P0[3][2] = 0;
    P0[3][3] = 0;

    X0[0] = 0;
    X0[1] = 0;
    X0[2] = 0;
    X0[3] = 0;

    U[0] = 0;
    U[1] = 0;
    U[2] = 0;
    U[3] = 0;

    kalmanFilterJ_SVI.setFixed(A, H, Q, R, B);

    init = false;

}

bool Kalman_SVI::isInitialized() const {
    return init;
}


void Kalman_SVI::setInitial(double X, double Y, double V, double H) {

    Kalman_SVI();

    P0[0][0] = 1;
    P0[0][1] = 0;
    P0[0][2] = 0;
    P0[0][3] = 0;

    P0[1][0] = 0;
    P0[1][1] = 1;
    P0[1][2] = 0;
    P0[1][3] = 0;

    P0[2][0] = 0;
    P0[2][1] = 0;
    P0[2][2] = 1;
    P0[2][3] = 0;

    P0[3][0] = 0;
    P0[3][1] = 0;
    P0[3][2] = 0;
    P0[3][3] = 1;

    X0[0] = X;
    X0[1] = V;
    X0[2] = Y;
    X0[3] = H;

    kalmanFilterJ_SVI.setInitial(X0, P0);

    init = true;
}

void Kalman_SVI::setT(double T) {

    A[0][0] = 1;
    A[0][1] = T;
    A[0][2] = 0;
    A[0][3] = 0;

    A[1][0] = 0;
    A[1][1] = 1;
    A[1][2] = 0;
    A[1][3] = 0;

    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = 1;
    A[2][3] = T;

    A[3][0] = 0;
    A[3][1] = 0;
    A[3][2] = 0;
    A[3][3] = 1;

    B[0][0] = pow(T, 2.0f) / 2.0f;
    B[1][0] = T;
    B[2][0] = pow(T, 2.0f) / 2.0f;
    B[3][0] = T;

    kalmanFilterJ_SVI.setA(A);
    kalmanFilterJ_SVI.setB(B);
}

void Kalman_SVI::setConfidence(double CX, double CY, double CVX, double CVY) {

    R[0][0] = CX;
    R[0][1] = 0;
    R[0][2] = 0;
    R[0][3] = 0;

    R[1][0] = 0;
    R[1][1] = CVX;
    R[1][2] = 0;
    R[1][3] = 0;

    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = CY;
    R[2][3] = 0;

    R[3][0] = 0;
    R[3][1] = 0;
    R[3][2] = 0;
    R[3][3] = CVY;

    kalmanFilterJ_SVI.setR(R);
}

void
Kalman_SVI::getDeltaPos(double T, double X, double Y, double V, double H, double CX, double CY, double CVX, double CVY,
                        double *Delta) {

    setT(T);
    setConfidence(CX, CY, CVX, CVY);

    X0[0] = X;
    X0[1] = V;
    X0[2] = Y;
    X0[3] = H;

    kalmanFilterJ_SVI.predict();
    kalmanFilterJ_SVI.correct(X0);

    Delta[0] = fabs(kalmanFilterJ_SVI.X[0] - X0[0]);
    Delta[1] = fabs(kalmanFilterJ_SVI.X[1] - X0[1]);
    Delta[2] = fabs(kalmanFilterJ_SVI.X[2] - X0[2]);
    Delta[3] = fabs(kalmanFilterJ_SVI.X[3] - X0[3]);
}


void
Kalman_SVI::getDeltaPos(double T, double X, double Y, double V, double H, double AX, double AY, double CX, double CY,
                        double CV, double CH, double *Delta) {

    setT(T);
    setConfidence(CX, CY, CV, CH);

    X0[0] = X;
    X0[1] = V;
    X0[2] = Y;
    X0[3] = H;

    U[0] = AX;
    U[1] = AX;
    U[2] = AY;
    U[3] = AY;

    kalmanFilterJ_SVI.predict(U);
    kalmanFilterJ_SVI.correct(X0);

    Delta[0] = fabs(kalmanFilterJ_SVI.X[0] - X0[0]);
    Delta[1] = fabs(kalmanFilterJ_SVI.X[1] - X0[1]);
    Delta[2] = fabs(kalmanFilterJ_SVI.X[2] - X0[2]);
    Delta[3] = fabs(kalmanFilterJ_SVI.X[3] - X0[3]);
}


