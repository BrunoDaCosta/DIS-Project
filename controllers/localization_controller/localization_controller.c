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
#define VERBOSE_ENC false
#define VERBOSE_ACC true
#define VERBOSE_POS true

/*CONSTANTES*/
#define WHEEL_AXIS 	0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020		// Radius of the wheel in meter

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
  double speed_odo[2];
  double acc_odo[2];
  
} measurement_t;



typedef struct
{
  pose_t pos;
  pose_t speed;
  pose_t acc;
  int id;
} robot_t;



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

static double KF_cov[MMS][MMS]={{0.001, 0, 0, 0},
                                {0, 0.001, 0, 0},
                                {0, 0, 0.001, 0},
                                {0, 0, 0, 0.001}};
//static motor_t        _motor = {false, true, true, MAX_SPEED / 4.0};
double last_gps_time_s = 0.0f;


//static FILE *fp;

//-----------------------------------------------------------------------------------//


static void controller_get_acc();
static void controller_get_encoder();
static void controller_get_gps();

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



void print_data(){
  int i;
  
  printf("\n\n\nCov. Matrix\n");
  for (i=0; i<4; i++)
    printf("%g %g %g %g\n", KF_cov[i][0], KF_cov[i][1], KF_cov[i][2], KF_cov[i][3]);
    

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
  
  print_matrix(KF_cov, 4,4);
                                
  mult(A,KF_cov,A_cov,4,4,4,4);
  transp(A,AT,4,4);
  mult(A_cov, AT, A_cov_AT, 4,4,4,4);
  
  scalar_mult(ts, R, ts_R, 4, 4);
  add(A_cov_AT, ts_R, KF_cov, 4,4,4,4);
  
  print_matrix(KF_cov, 4,4);
}


void Kalman_Filter(){

  printf("Before\n");
  printf("Cov matrix\n");
  print_matrix(KF_cov, 4,4);

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
  
  _robot.pos.x   = X_new[0][0];
  _robot.pos.y   = X_new[1][0];
  _robot.speed.x = X_new[2][0];
  _robot.speed.y = X_new[3][0];
  
  
  
  printf("After\n");
  printf("Cov matrix\n");
  print_matrix(KF_cov, 4,4);

  printf("X matrix\n");
  print_matrix(X_new, 4,1);
    
}



int main() 
{ 
  _robot.pos.x = -2.9;
  _robot.pos.y = 0;
  _robot.pos.heading = -1.5708;
  _robot.speed.x = 0;
  _robot.speed.y = 0;
  
  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);

  while (wb_robot_step(time_step) != -1)  {
    controller_get_encoder();
    controller_get_acc();
    
    if (VERBOSE_POS)  printf("ROBOT pose: %g %g\n", _robot.pos.x , _robot.pos.y);
    
    KF_Update_Cov_Matrix((double) time_step/1000);
     
    double time_now_s = wb_robot_get_time();
    if (time_now_s - last_gps_time_s >= 1.0f) {
      last_gps_time_s = time_now_s;
      controller_get_gps();
      
      Kalman_Filter();
      
      if (VERBOSE_POS)  printf("ROBOT pose after Kalman: %g %g\n", _robot.pos.x , _robot.pos.y);
    } 

   
  // Use one of the two trajectories.
    trajectory_1(dev_left_motor, dev_right_motor);
//    trajectory_2(dev_left_motor, dev_right_motor); 
  }
  
}


/**
 * @brief      Read the encoders values from the sensors
 */
void controller_get_encoder()
{
  int time_step = wb_robot_get_basic_time_step();
  // Store previous value of the left encoder
  _meas.prev_left_enc = _meas.left_enc;

  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  
  double deltaleft=_meas.left_enc-_meas.prev_left_enc;
  // Store previous value of the right encoder
  _meas.prev_right_enc = _meas.right_enc;
  
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
  
  double deltaright=_meas.right_enc-_meas.prev_right_enc;
  
  deltaleft  *= WHEEL_RADIUS;
  deltaright *= WHEEL_RADIUS;
  
  double omega = ( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step );
  double speed = ( deltaright + deltaleft ) / ( 2.0 * time_step );

  double a = _robot.pos.heading;

  double speed_wx = speed * cos(a);
  double speed_wy = speed * sin(a);
  
  //Enregistrement vitesse pour Kalman
  _meas.speed_odo[0]=speed_wx;
  _meas.speed_odo[1]=speed_wy;
  
  //A enlever à terme
  _robot.pos.x += speed_wx * time_step;
  _robot.pos.y += speed_wy * time_step;
  _robot.pos.heading += omega * time_step;
  
  if(VERBOSE_ENC)
    printf("ROBOT enc : vx: %g  vy: %g heading: %g\n", speed_wx, speed_wy, a);
}

void controller_get_acc()
{
  int time_step = wb_robot_get_basic_time_step();
  // To Do : Call the function to get the accelerometer measurements. Uncomment and complete the following line. Note : Use _robot.acc
  const double * acc_values = wb_accelerometer_get_values(dev_acc);

  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));
  double acc = ( _meas.acc[1] - _meas.acc_mean[1]);
  

  _meas.acc_odo[0] = acc * cos(_robot.pos.heading) * time_step;
  _meas.acc_odo[1] = acc * sin(_robot.pos.heading) * time_step;
  //memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
  
  if(VERBOSE_ACC)
    printf("ROBOT acc : ax: %g  ay: %g heading: %g\n", _meas.acc_odo[0], _meas.acc_odo[1], _robot.pos.heading);	
  

}

/**
 * @brief     Get the gps measurements for the position of the robot. Get the heading angle. Fill the pose structure. 
 */
void controller_get_gps(){
  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  const double * gps_position = wb_gps_get_values(dev_gps);
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps)); 
}


