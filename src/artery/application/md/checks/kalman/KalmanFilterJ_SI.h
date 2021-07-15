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

#ifndef __VEINS_KalmanFilterJ_SI
#define __VEINS_KalmanFilterJ_SI
#include <iostream>
#include "MatrixOp_SI.h"
using namespace std;

class KalmanFilterJ_SI {

	public:

		int n;
		int m; 

		double A[KLM_N_SI][KLM_N_SI];
		double AT[KLM_N_SI][KLM_N_SI]; 

		double B[KLM_N_SI][KLM_M_SI]; 

		double H[KLM_N_SI][KLM_N_SI]; 
		double HT[KLM_N_SI][KLM_N_SI]; 
		double Q[KLM_N_SI][KLM_N_SI];
		double R[KLM_N_SI][KLM_N_SI];
		double I[KLM_N_SI][KLM_N_SI]; 

		double X[KLM_N_SI]; 
		double P[KLM_N_SI][KLM_N_SI]; 
		double K[KLM_N_SI][KLM_N_SI]; 

		double X0[KLM_N_SI]; 
		double P0[KLM_N_SI][KLM_N_SI]; 

		double Temp_1[KLM_N_SI];
		double Temp_2[KLM_N_SI]; 
		double Temp_3[KLM_N_SI]; 


		double TempN_1[KLM_N_SI][KLM_N_SI]; 
		double TempN_2[KLM_N_SI][KLM_N_SI]; 
		double TempN_3[KLM_N_SI][KLM_N_SI]; 
		double TempN_4[KLM_N_SI][KLM_N_SI]; 

		MatrixOp_SI matrixOp_SI = MatrixOp_SI();
		

		KalmanFilterJ_SI();

		void setFixed ( double _A [][KLM_N_SI], double  _H [][KLM_N_SI], double _Q [][KLM_N_SI], double _R [][KLM_N_SI]);

		void setFixed ( double _A [][KLM_N_SI], double  _H [][KLM_N_SI], double _Q [][KLM_N_SI], double _R [][KLM_N_SI] , double _B [][KLM_M_SI]);


		void setA(double _A [][KLM_N_SI]);

		void setB(double _B [][KLM_M_SI]);

		void setR(double _R [][KLM_N_SI]);

		void setInitial( double  _X0[], double _P0[][KLM_N_SI] );

		void predict ( void );

		void predict( double  U []);
		void correct ( double Z []);

};

#endif



