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

#ifndef __VEINS_KalmanFilterJ_SVI
#define __VEINS_KalmanFilterJ_SVI
#include <iostream>
#include "MatrixOp_SVI.h"
using namespace std;

class KalmanFilterJ_SVI {

	public:

		int n;
		int m; 

		double A[KLM_N_SVI][KLM_N_SVI];
		double AT[KLM_N_SVI][KLM_N_SVI]; 

		double B[KLM_N_SVI][KLM_M_SVI]; 

		double H[KLM_N_SVI][KLM_N_SVI]; 
		double HT[KLM_N_SVI][KLM_N_SVI]; 
		double Q[KLM_N_SVI][KLM_N_SVI];
		double R[KLM_N_SVI][KLM_N_SVI];
		double I[KLM_N_SVI][KLM_N_SVI]; 

		double X[KLM_N_SVI]; 
		double P[KLM_N_SVI][KLM_N_SVI]; 
		double K[KLM_N_SVI][KLM_N_SVI]; 

		double X0[KLM_N_SVI]; 
		double P0[KLM_N_SVI][KLM_N_SVI]; 

		double Temp_1[KLM_N_SVI];
		double Temp_2[KLM_N_SVI]; 
		double Temp_3[KLM_N_SVI]; 

		double TempN_1[KLM_N_SVI][KLM_N_SVI]; 
		double TempN_2[KLM_N_SVI][KLM_N_SVI]; 
		double TempN_3[KLM_N_SVI][KLM_N_SVI]; 
		double TempN_4[KLM_N_SVI][KLM_N_SVI]; 

		MatrixOp_SVI matrixOp_SVI = MatrixOp_SVI();
		

		KalmanFilterJ_SVI();

		void setFixed ( double _A [][KLM_N_SVI], double  _H [][KLM_N_SVI], double _Q [][KLM_N_SVI], double _R [][KLM_N_SVI]);

		void setFixed ( double _A [][KLM_N_SVI], double  _H [][KLM_N_SVI], double _Q [][KLM_N_SVI], double _R [][KLM_N_SVI] , double _B [][KLM_M_SVI]);

		void setA(double _A [][KLM_N_SVI]);

		void setB(double _B [][KLM_M_SVI]);

		void setR(double _R [][KLM_N_SVI]);

		void setInitial( double  _X0[], double _P0[][KLM_N_SVI] );

		void predict ( void );

		void predict( double  U []);
		void correct ( double Z []);

};

#endif



