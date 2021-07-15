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

#ifndef __VEINS_KalmanFilterJ_SC
#define __VEINS_KalmanFilterJ_SC
#include <iostream>
#include "MatrixOp_SC.h"
using namespace std;

class KalmanFilterJ_SC {

	public:

		int n;
		int m; 

		double A[KLM_N_SC][KLM_N_SC];
		double AT[KLM_N_SC][KLM_N_SC]; 

		double B[KLM_N_SC][KLM_M_SC]; 

		double H[KLM_N_SC][KLM_N_SC]; 
		double HT[KLM_N_SC][KLM_N_SC]; 
		double Q[KLM_N_SC][KLM_N_SC];
		double R[KLM_N_SC][KLM_N_SC];
		double I[KLM_N_SC][KLM_N_SC]; 

		double X[KLM_N_SC]; 
		double P[KLM_N_SC][KLM_N_SC]; 
		double K[KLM_N_SC][KLM_N_SC]; 

		double X0[KLM_N_SC]; 
		double P0[KLM_N_SC][KLM_N_SC]; 

		double Temp_1[KLM_N_SC];
		double Temp_2[KLM_N_SC]; 
		double Temp_3[KLM_N_SC]; 


		double TempN_1[KLM_N_SC][KLM_N_SC]; 
		double TempN_2[KLM_N_SC][KLM_N_SC]; 
		double TempN_3[KLM_N_SC][KLM_N_SC]; 
		double TempN_4[KLM_N_SC][KLM_N_SC]; 

		MatrixOp_SC matrixOp_SC = MatrixOp_SC();
		

		KalmanFilterJ_SC();

		void setFixed ( double _A [][KLM_N_SC], double  _H [][KLM_N_SC], double _Q [][KLM_N_SC], double _R [][KLM_N_SC]);

		void setFixed ( double _A [][KLM_N_SC], double  _H [][KLM_N_SC], double _Q [][KLM_N_SC], double _R [][KLM_N_SC] , double _B [][KLM_M_SC]);


		void setA(double _A [][KLM_N_SC]);

		void setB(double _B [][KLM_M_SC]);

		void setR(double _R [][KLM_N_SC]);

		void setInitial( double  _X0[], double _P0[][KLM_N_SC] );

		void predict ( void );

		void predict( double  U []);
		void correct ( double Z []);

};

#endif



