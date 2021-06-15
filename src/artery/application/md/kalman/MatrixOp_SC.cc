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
#include "MatrixOp_SC.h"
using namespace std;

MatrixOp_SC::MatrixOp_SC(){   
}

void MatrixOp_SC::multiply(double  M1[][KLM_N_SC], double M2[][KLM_N_SC], double  Mret[][KLM_N_SC], int r1,  int c2,  int c1){
        for(int i=0; i<r1; ++i)
         for(int j=0; j<c2; ++j){
            Mret[i][j]=0.0;
        }
      for(int i=0; i<r1; ++i)
         for(int j=0; j<c2; ++j)
            for(int k=0; k<c1; ++k) {
               Mret[i][j]+=M1[i][k]*M2[k][j];
            }
}

void MatrixOp_SC::multiply1D(double M1[][KLM_M_SC], double M2[], double Mret[], int r){
      for(int i=0; i<r; ++i)
        Mret[i]=M1[i][0]*M2[i];
}

void MatrixOp_SC::multiply21D(double  M1[][KLM_N_SC], double * M2, double * Mret, int r1,  int c1){
        for(int k=0; k<r1; ++k) {
            Mret[k]=0;
        }

      for(int i=0; i<r1; ++i)
        for(int k=0; k<c1; ++k) {
            Mret[i]+=M1[i][k]*M2[k];
        }
}

void MatrixOp_SC::copy(double M[][KLM_N_SC], double Mret[][KLM_N_SC], int r,  int c){
      for(int i=0; i<r; ++i){
            memcpy(M[i], Mret[i], c*sizeof(double));
      }
}

void MatrixOp_SC::copyM(double M[][KLM_M_SC], double Mret[][KLM_M_SC], int r,  int c){
      for(int i=0; i<r; ++i){
            memcpy(M[i], Mret[i], c*sizeof(double));
      }
}

void MatrixOp_SC::copy(double * M, double * Mret, int r){
    memcpy(M, Mret, r*sizeof(double));
}

void MatrixOp_SC::add(double  M1[][KLM_N_SC], double  M2[][KLM_N_SC], double  Mret[][KLM_N_SC], int r,  int c){
      for(int i=0; i<r; ++i)
         for(int j=0; j<c; ++j){
               Mret[i][j]=M1[i][j]+M2[i][j];
            }
}
void MatrixOp_SC::add1D(double * M1, double * M2, double * Mret, int r){
      for(int i=0; i<r; ++i){
            Mret[i]=M1[i]+M2[i];
        }
}

void MatrixOp_SC::substract(double M1[][KLM_N_SC], double M2[][KLM_N_SC], double  Mret[][KLM_N_SC], int r,  int c){
      for(int i=0; i<r; ++i)
         for(int j=0; j<c; ++j){
               Mret[i][j]=M1[i][j]-M2[i][j];
            }
}

void MatrixOp_SC::substract1D(double * M1, double * M2, double * Mret, int r){
      for(int i=0; i<r; ++i){
           Mret[i]=M1[i]-M2[i];
        }
}

void MatrixOp_SC::transpose(double M[][KLM_N_SC], double Mret[][KLM_N_SC],  int r,  int c) {
    for(int i = 0; i < r; ++i)
        for(int j = 0; j < c; ++j)
        {
            Mret[j][i]=M[i][j];
        }
}

void MatrixOp_SC::cofactor(double ** A, double ** temp, int p, int q, int n) 
{ 
    int i = 0, j = 0; 
  
    // Looping for each element of the matrix 
    for (int row = 0; row < n; row++) 
    { 
        for (int col = 0; col < n; col++) 
        { 
            //  Copying into temporary matrix only those element 
            //  which are not in given row and column 
            if (row != p && col != q) 
            { 
                temp[i][j++] = A[row][col]; 
  
                // Row is filled, so increase row index and 
                // reset col index 
                if (j == n - 1) 
                { 
                    j = 0; 
                    i++; 
                } 
            } 
        } 
    } 
} 
  
double MatrixOp_SC::determinant(double ** A, int n) 
{ 
    double D = 0; // Initialize result 
    double D_temp = 0;
  
    //  Base case : if matrix contains single element 
    if (n == 1) 
        return A[0][0]; 

    double ** temp;
    temp = new double*[n]; //Kalman Gain matrix 
        for(int i = 0; i < n; ++i)
            temp[i] = new double[n];

    int sign = 1;  // To store sign multiplier 
  
     // Iterate for each element of first row 
    for (int f = 0; f < n; f++) 
    { 
        // Getting Cofactor of A[0][f] 
        cofactor(A, temp, 0, f, n); 

        D_temp = A[0][f] * determinant(temp, n - 1);
        if(D_temp!=0.0){
            D_temp = sign * D_temp;
        }

        D += D_temp; 
  
        // terms are to be added with alternate sign 
        sign = -sign; 
    }
  
    for(int i = 0; i < n; i++)
      delete[] temp[i];
    delete[] temp;

    return D; 
}
  
// Function to get adjoint of A[N][N] in adj[N][N]. 
void MatrixOp_SC::adjoint(double ** A, double ** adj, int N) 
{ 
    if (N == 1) 
    { 
        adj[0][0] = 1; 
        return; 
    } 
    double ADJ_temp = 0;
  
    // temp is used to store cofactors of A[][] 
    int sign = 1;
    double ** temp;
    temp = new double*[N]; //Kalman Gain matrix 
        for(int i = 0; i < N; ++i)
            temp[i] = new double[N];

  
    for (int i=0; i<N; i++) 
    { 
        for (int j=0; j<N; j++) 
        { 
            // Get cofactor of A[i][j] 
            cofactor(A, temp, i, j, N); 
  
            // sign of adj[j][i] positive if sum of row 
            // and column indexes is even. 


            sign = ((i+j)%2==0)? 1: -1; 
  
            // Interchanging rows and columns to get the 
            // transpose of the cofactor matrix 

            ADJ_temp = (determinant(temp, N-1));
            if(ADJ_temp!=0.0){
                ADJ_temp = (sign)*ADJ_temp;
            }
            adj[j][i] = ADJ_temp; 
        } 
    }
    for(int i = 0; i < N; i++)
      delete[] temp[i];
    delete[] temp;
} 
  
// Function to calculate and store inverse, returns false if 
// matrix is singular 
void MatrixOp_SC::inverse(double A[][KLM_N_SC], double inverse[][KLM_N_SC], int N) 
{ 
    // Find determinant of A[][] 
    double ** A_det;
    A_det = new double*[N]; //Kalman Gain matrix 
        for(int i = 0; i < N; ++i){
            A_det[i] = new double[N];
            for(int j = 0; j < N; ++j){
                A_det[i][j] = A[i][j];
            }   
        }

    double det = determinant(A_det, N);
    if (det == 0) 
    { 
        cout << "Singular matrix, can't find its inverse"; 
        exit(0);
    } 
  
    // Find adjoint 
    double ** adj;
    adj = new double*[N]; //Kalman Gain matrix 
        for(int i = 0; i < N; ++i)
            adj[i] = new double[N];

    adjoint(A_det, adj, N); 
  
    // Find Inverse using formula "inverse(A) = adj(A)/det(A)" 
    for (int i=0; i<N; i++) 
        for (int j=0; j<N; j++) 
            inverse[i][j] = adj[i][j]/det; 

    for(int i = 0; i < N; i++)
      delete[] A_det[i];
    delete[] A_det;

    for(int i = 0; i < N; i++)
      delete[] adj[i];
    delete[] adj;
} 

void MatrixOp_SC::printMat(std::string Sym, double A[][KLM_N_SC], int r, int c){
    cout << "----------"<<Sym<< endl;
    for(int i = 0; i<r;i++){
        for(int j = 0; j<c;j++){
            cout << A[i][j] << " ";
        }
        cout << endl;
    }
}
void MatrixOp_SC::printVec(std::string Sym, double A[], int r){
    cout << "----------"<<Sym<< endl;
    for(int i = 0; i<r;i++){
      cout << A[i] << " ";
    }
    cout << endl;
}
