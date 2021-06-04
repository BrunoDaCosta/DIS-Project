#ifndef KALMAN_H
#define KALMAN_H

#define MMS 4 //Matrix max size
#define VERBOSE_KF 0

void KF_Update_Cov_Matrix(double ts);
void Kalman_Filter(double* pos_x, double* pos_y, double* heading, double* speed_x, double* speed_y, double* GPS_x, double* GPS_y);

void print_cov_matrix();

#endif
