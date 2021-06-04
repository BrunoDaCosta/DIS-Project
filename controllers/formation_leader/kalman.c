#include "kalman.h"
#include <stdio.h>
#include <math.h>

static void add(double a[][MMS],double b[][MMS],double res[][MMS],int r1,int c1,int r2,int c2);
static void transp(double a[][MMS],double res[][MMS], int r1, int c1);
static void scalar_mult(double scalar, double a[][MMS], double res[][MMS], int r1, int c1);
static void mult(double a[][MMS],double b[][MMS],double res[][MMS],int r1,int c1,int r2,int c2);
static void inv(double a[][MMS], double res[][MMS]);
static void copy_matrix(double a[MMS][MMS], double res[MMS][MMS], int r1, int c1);
static void print_matrix(double a[][MMS], int row, int col);

double KF_cov[MMS][MMS]={{0.001, 0, 0, 0},
                         {0, 0.001, 0, 0},
                         {0, 0, 0.001, 0},
                         {0, 0, 0, 0.001}};

//---------------- External functions ---------------

void KF_Update_Cov_Matrix(double ts){
    double A[MMS][MMS]={{1, 0, ts, 0},
                        {0, 1, 0, ts},
                        {0, 0, 1, 0 },
                        {0, 0, 0, 1 }};

    double R[MMS][MMS]={{0.05, 0, 0, 0},
                        {0, 0.05, 0, 0},
                        {0, 0, 0.01, 0},
                        {0, 0, 0, 0.01}};

    double A_cov[MMS][MMS];
    double AT[MMS][MMS];
    double A_cov_AT[MMS][MMS];
    double ts_R[MMS][MMS];

    mult(A,KF_cov,A_cov,4,4,4,4);
    transp(A,AT,4,4);
    mult(A_cov, AT, A_cov_AT, 4,4,4,4);

    scalar_mult(ts, R, ts_R, 4, 4);
    add(A_cov_AT, ts_R, KF_cov, 4,4,4,4);
}

void Kalman_Filter(double* pos_x, double* pos_y, double* speed_x, double* speed_y, double* GPS_x, double* GPS_y){
    static double X[MMS][MMS];
    X[0][0]=*pos_x; X[1][0]=*pos_y;
    X[2][0]=*speed_x; X[3][0]=*speed_y;

    if (VERBOSE_KF){
        printf("______________________________________________\n");
        printf("Before\n");
        printf("Cov matrix\n");
        print_matrix(KF_cov, 4,4);
        printf("X matrix\n");
        print_matrix(X, 4,1);
    }

    static double C[MMS][MMS]={{1, 0, 0, 0},
                             {0, 1, 0, 0}};
    static double Q[MMS][MMS]={{1, 0},{0, 1}};

    static double Z[MMS][MMS];
    Z[0][0] = *GPS_x; //_meas.gps[0];
    Z[1][0] = *GPS_y; //_meas.gps[2];

    static double X_new[MMS][MMS];

    static double K[MMS][MMS];
    static double temp1[MMS][MMS];
    static double temp2[MMS][MMS];
    static double cov_Ct[MMS][MMS];
    static double eye4[MMS][MMS]={{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}};

    transp(C, temp1, 2, 4);
    mult(KF_cov,temp1,cov_Ct,4,4,4,2);
    mult(C,cov_Ct,temp1,2,4,4,2);
    add(temp1, Q, temp2, 2,2,2,2);
    inv(temp2, temp1);
    mult(cov_Ct,temp1,K,4,2,2,2);

    mult(C, X, temp2, 2,4,4,1);
    scalar_mult(-1, temp2, temp1, 2,1);
    add(Z, temp1, temp2, 2,1,2,1);
    mult(K, temp2, temp1, 4,2,2,1);
    add(X, temp1, X_new, 4,1,4,1);

    mult(K,C,temp1, 4,2,2,4);
    scalar_mult(-1, temp1, temp2, 4,4);
    add(eye4, temp2, temp1, 4,4,4,4);
    mult(KF_cov, temp1, temp2, 4,4,4,4);
    copy_matrix(temp2, KF_cov, 4,4);

    *pos_x   = X_new[0][0];
    *pos_y   = X_new[1][0];
    *speed_x = X_new[2][0];
    *speed_y = X_new[3][0];

    if (VERBOSE_KF){
        printf("After\n");
        printf("Cov matrix\n");
        print_matrix(KF_cov, 4,4);

        printf("X matrix\n");
        print_matrix(X_new, 4,1);

        printf("Position from Kalman: %g %g\n", X_new[0][0], X_new[1][0]);
    }
}

void print_cov_matrix(){
    print_matrix(KF_cov, 4, 4);
}


// -------------- Matrix operations ----------------

void add(double a[][MMS],double b[][MMS],double res[][MMS],int r1,int c1,int r2,int c2)
{
  if (c1!=c2 || r1!=r2){
    printf("Error: Matrix size don't match");
    return;
  }

  int i,j;
  for(i=0; i<r1; ++i)
    for(j=0; j<c1; ++j)
      res[i][j]=a[i][j]+b[i][j];
}

void transp(double a[][MMS],double res[][MMS], int r1, int c1){
  int i,j;
  for(i=0; i<r1; ++i)
    for(j=0; j<c1; ++j)
      res[j][i]=a[i][j];
}

void scalar_mult(double scalar, double a[][MMS], double res[][MMS], int r1, int c1){
  int i,j;
  for(i=0; i<r1; ++i)
    for(j=0; j<c1; ++j)
      res[i][j]=scalar*a[i][j];
}

void mult(double a[][MMS],double b[][MMS],double res[][MMS],int r1,int c1,int r2,int c2){
    if (c1!=r2){
      printf("Error: Matrix size don't match");
      return;
    }

    int i,j,k;
    /* Initializing elements of matrix mult to 0.*/
    for(i=0; i<r1; ++i)
        for(j=0; j<c2; ++j)
            res[i][j]=0;

    /* Multiplying matrix a and b and storing in array mult. */
    for(i=0; i<r1; ++i)
        for(j=0; j<c2; ++j)
            for(k=0; k<c1; ++k)
                res[i][j]+=a[i][k]*b[k][j];
}


void inv(double a[][MMS], double res[][MMS]){
  double det=1./((double) (a[0][0]*a[1][1]-a[0][1]*a[1][0]));
  res[0][0]=a[1][1]*det;
  res[1][1]=a[0][0]*det;
  res[1][0]=-a[1][0]*det;
  res[0][1]=-a[0][1]*det;

  int i,j;
  for (i=0; i<2; i++){
    for (j=0; j<2; j++){
      if (fabs(res[i][j])<1E-7){
        res[i][j]=0;
      }
    }
  }

}

void copy_matrix(double a[MMS][MMS], double res[MMS][MMS], int r1, int c1){
  int i,j;
  for(i=0; i<r1; ++i)
    for(j=0; j<c1; ++j)
      res[i][j]=a[i][j];

}


void print_matrix(double a[][MMS], int row, int col)
{
    int i, j;
    printf("[");
    for(i=0; i<row; ++i){
        for(j=0; j<col; ++j){
            printf("%g  ",a[i][j]);
        }
        printf(";\n");
    }
    printf("]\n");
}
