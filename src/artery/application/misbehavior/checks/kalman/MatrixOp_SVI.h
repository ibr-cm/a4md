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
#ifndef __VEINS_MatrixOp_SVI
#define __VEINS_MatrixOp_SVI

#include <algorithm>
#include <memory.h>
#include <iostream>

#define KLM_N_SVI 4
#define KLM_M_SVI 1

class MatrixOp_SVI {

private:
	double determinant(double ** A, int n);
	void adjoint(double ** A, double ** adj, int N);
public:
	MatrixOp_SVI();
	void multiply(double M1[][KLM_N_SVI], double M2[][KLM_N_SVI], double Mret[][KLM_N_SVI], int r1,  int c2,  int c1);

	void multiply1D(double M1[][KLM_M_SVI], double M2[], double Mret[], int r);

	void multiply21D(double M1[][KLM_N_SVI], double * M2, double * Mret, int r1,  int c1);
	
	void add(double M1[][KLM_N_SVI], double M2[][KLM_N_SVI], double Mret[][KLM_N_SVI], int r,  int c);
	void add1D(double * M1, double * M2, double * Mret, int r);
	
	void substract(double M1[][KLM_N_SVI], double M2[][KLM_N_SVI], double Mret[][KLM_N_SVI], int r,  int c);
	void substract1D(double * M1, double * M2, double * Mret, int r);

	void transpose(double M[][KLM_N_SVI], double  Mret[][KLM_N_SVI],  int r,  int c);
	void cofactor(double ** A, double ** temp, int p, int q, int n);
	void inverse(double  A[][KLM_N_SVI], double inverse[][KLM_N_SVI], int N);

	void copy(double  M[][KLM_N_SVI], double  Mret[][KLM_N_SVI], int r,  int c);
	void copyM(double M[][KLM_M_SVI], double  Mret[][KLM_M_SVI], int r,  int c);

	void copy(double * M, double * Mret, int r);

	void printMat(std::string Sym, double A [][KLM_N_SVI], int r, int c);
	void printVec(std::string Sym, double * A, int r);
};
#endif




