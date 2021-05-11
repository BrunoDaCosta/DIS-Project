#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

#include "trajectories.h"
#include "odometry.h"

#include "utils.h"


#define VERBOSE_GPS false
#define VERBOSE_ACC true
#define VERBOSE_ENC false

typedef struct 
{
  double prev_gps[3];
  double gps[3];
  double acc_mean[3];
  double acc[3];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
} measurement_t;



typedef struct
{
  pose_t pos;
  pose_t speed;
  pose_t acc;
  int id;
} robot_t;

typedef struct
{
  double X[MMS][MMS];
  double new_X[MMS][MMS];
  double cov[4][4];    //Covariance matrix for KF
  double new_cov[4][4];
  double R[4][4];      // R matrix for KF
  double Q[2][2];      // Q matrix for KF 
} kalman_filter_param;

WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor; 
WbDeviceTag dev_right_motor; 

//-----------------------------------------------------------------------------------//
/*VARIABLES*/
static measurement_t  _meas;
static robot_t        _robot;
static pose_t         _robot_init_pos;

static kalman_filter_param _KF;
//static motor_t        _motor = {false, true, true, MAX_SPEED / 4.0};
double last_gps_time_s = 0.0f;


//static FILE *fp;

//-----------------------------------------------------------------------------------//


static void controller_get_acc();
static void controller_get_encoder();

void init_devices(int ts);

void init_devices(int ts) {
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps, 1000);
  //wb_gps_enable(dev_gps, 1);
  
  dev_acc = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(dev_acc, ts);
  
  
  dev_left_encoder = wb_robot_get_device("left wheel sensor");
  dev_right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(dev_left_encoder,  ts);
  wb_position_sensor_enable(dev_right_encoder, ts);

  dev_left_motor = wb_robot_get_device("left wheel motor");
  dev_right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(dev_left_motor, INFINITY);
  wb_motor_set_position(dev_right_motor, INFINITY);
  wb_motor_set_velocity(dev_left_motor, 0.0);
  wb_motor_set_velocity(dev_right_motor, 0.0);
}

/**
 * @brief     Get the gps measurements for the position of the robot. Get the heading angle. Fill the pose structure. 
 */
 void controller_get_pose(){
  //double time_now_s = wb_robot_get_time();

  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  const double * gps_position = wb_gps_get_values(dev_gps);
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));

  //if(VERBOSE_GPS)
 // printf("ROBOT gps is at position: %g %g %g\n", _meas.gps[0], _meas.gps[1], _meas.gps[2]);
   /* 
  if (time_now_s - last_gps_time_s > 0.1f) {
    last_gps_time_s = time_now_s;
    controller_get_gps();

    // To Do : Fill the structure pose_t {x, y, heading}. Use the _pose_origin.
    _pose.x = _meas.gps[0] - _pose_origin.x;
    
    _pose.y = -(_meas.gps[2] - _pose_origin.y);
    
    _pose.heading = controller_get_heading() + _pose_origin.heading;
  
    if(VERBOSE_POSE)
      printf("ROBOT pose : %g %g %g\n", _pose.x , _pose.y , RAD2DEG(_pose.heading));
  }*/
  
}


void init_Kalman_Filter(){
  int i, j;
  for (i=0; i<4; i++){
    for (j=0; j<4; j++){
      _KF.cov[i][j]=0;
      _KF.R[i][j]=0;
    }
  }
  _KF.cov[0][0]=0.001; _KF.cov[1][1]=0.001; _KF.cov[2][2]=0.001; _KF.cov[3][3]=0.001;
  _KF.R[0][0]=0.05; _KF.R[1][1]=0.05; _KF.R[2][2]=0.01; _KF.R[3][3]=0.01;
  
  for (i=0; i<2; i++){
    for (j=0; j<2; j++){
      _KF.Q[i][j]=0;
    }
  }          
  _KF.Q[0][0]=1; _KF.Q[1][1]=1;        
}

void print_data(){
  int i;
  
  printf("\n\n\nCov. Matrix\n");
  for (i=0; i<4; i++)
    printf("%g %g %g %g\n", _KF.cov[i][0], _KF.cov[i][1], _KF.cov[i][2], _KF.cov[i][3]);
    

  printf("Robot data\n");
  printf("%f %f %f %f\n", _robot.pos.x, _robot.pos.y, _robot.speed.x, _robot.speed.y);
  printf("%f %f\n\n", _meas.gps[0], _meas.gps[2]);
  
  
}

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
  
  print_matrix(_KF.cov, 4,4);
                                
  mult(A,_KF.cov,A_cov,4,4,4,4);
  transp(A,AT,4,4);
  mult(A_cov, AT, A_cov_AT, 4,4,4,4);
  
  scalar_mult(ts, _KF.R, ts_R, 4, 4);
  add(A_cov_AT, ts_R, _KF.cov, 4,4,4,4);
  
  print_matrix(_KF.cov, 4,4);
}


void Kalman_Filter(double ts){

  printf("Before\n");
  printf("Cov matrix\n");
  print_matrix(_KF.cov, 4,4);

  static double X[MMS][MMS];
  X[0][0]=_robot.pos.x;
  X[1][0]=_robot.pos.y;
  X[2][0]=_robot.speed.x;
  X[3][0]=_robot.speed.y;

  printf("X matrix\n");
  print_matrix(X, 4,1);
                                
  static double C[MMS][MMS]={{1, 0, 0, 0},
                             {0, 1, 0, 0}};

  static double Q[MMS][MMS]={{1, 0},{0, 1}};

  static double Z[MMS][MMS];
  Z[0][0] = _meas.gps[0];
  Z[1][0] = _meas.gps[2];
  
  static double X_new[MMS][MMS];
  
  static double K[MMS][MMS];
  static double temp1[MMS][MMS];
  static double temp2[MMS][MMS];
  static double cov_Ct[MMS][MMS];
  static double eye4[MMS][MMS]={{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}};
  
  transp(C, temp1, 2, 4);
  mult(_KF.cov,temp1,cov_Ct,4,4,4,2);
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
  mult(_KF.cov, temp1, temp2, 4,4,4,4);
  copy_matrix(temp2, _KF.cov, 4,4);
  
  _robot.pos.x   = X_new[0][0];
  _robot.pos.y   = X_new[1][0];
  _robot.speed.x = X_new[2][0];
  _robot.speed.y = X_new[3][0];
  
  
  
  printf("After\n");
  printf("Cov matrix\n");
  print_matrix(_KF.cov, 4,4);

  printf("X matrix\n");
  print_matrix(X_new, 4,1);
    
}



int main() 
{

  init_Kalman_Filter();
  
  _robot.pos.x   = 45;
  _robot.pos.y   = 64.3;
  _robot.speed.x = 0.45;
  _robot.speed.y = -0.92;
  
  _meas.gps[0]=25.6;
  _meas.gps[2]=100.5;
  
  
  //KF_Update_Cov_Matrix(5.4);
  Kalman_Filter(5.4);
  return 0;
  
  
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);

  while (wb_robot_step(time_step) != -1)  {
    controller_get_acc();
    controller_get_encoder();
    
    Kalman_Filter((double) time_step/1000);
    print_data();
    //print_data();
//    trajectory_2(dev_left_motor, dev_right_motor);
   
  // Use one of the two trajectories.
    trajectory_1(dev_left_motor, dev_right_motor);
//    trajectory_2(dev_left_motor, dev_right_motor); 
    controller_get_pose();
    if(VERBOSE_GPS && 0)
      printf("ROBOT pose : %g %g\n", _robot.pos.x , _robot.pos.y);
  }
  
}

void controller_get_acc()
{
  // To Do : Call the function to get the accelerometer measurements. Uncomment and complete the following line. Note : Use _robot.acc
  const double * acc_values = wb_accelerometer_get_values(dev_acc);

  // To Do : Copy the acc_values into the measurment structure (use memcpy)
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  if(VERBOSE_ACC)
    printf("ROBOT acc : %g %g %g\n", _meas.acc[0], _meas.acc[1] , _meas.acc[2]);
}


/**
 * @brief      Read the encoders values from the sensors
 */
void controller_get_encoder()
{
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  
  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;
  
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);

  if(VERBOSE_ENC)
    printf("ROBOT enc : %g %g\n", _meas.left_enc, _meas.right_enc);
}

