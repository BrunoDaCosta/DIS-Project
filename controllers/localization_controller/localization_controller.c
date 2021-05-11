#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

#include "trajectories.h"
#include "odometry.h"


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

// A FAIRE: TRANSFORMER LA DIRECTION DE L'ACCELERATION DANS LES COORDONEE GLOBAL
void Kalman_Filter(double ts){
  int i, j;

  _robot.pos.x   = _robot.pos.x + _robot.speed.x*ts;
  _robot.pos.y   = _robot.pos.y + _robot.speed.y*ts;
  _robot.speed.x = _robot.speed.x + _meas.acc[0]*ts;
  _robot.speed.y = _robot.speed.y + _meas.acc[1]*ts;
  
  _KF.new_cov[0][0]=_KF.cov[0][0] + (_KF.cov[2][0]+_KF.cov[0][2])*ts + _KF.cov[2][2]*ts*ts + _KF.R[0][0]*ts;
  _KF.new_cov[0][2]=_KF.cov[0][2] + _KF.cov[2][2]*ts;
  _KF.new_cov[1][1]=_KF.cov[1][1] + (_KF.cov[1][3]+_KF.cov[3][1])*ts + _KF.cov[3][3]*ts*ts + _KF.R[1][1]*ts;
  _KF.new_cov[1][3]=_KF.cov[1][3] + _KF.cov[3][3]*ts;
  _KF.new_cov[2][0]=_KF.cov[2][0] + _KF.cov[2][2]*ts; 
  _KF.new_cov[2][2]=_KF.cov[2][2]  + _KF.R[2][2]*ts;
  _KF.new_cov[3][1]=_KF.cov[3][1] + _KF.cov[3][3]*ts;
  _KF.new_cov[3][3]=_KF.cov[3][3]  + _KF.R[3][3]*ts;
  
  
  
  for (i=0; i<4; i++){
    for (j=0; j<4; j++) _KF.cov[i][j]=_KF.new_cov[i][j];
  }
  
  double time_now_s = wb_robot_get_time();
  if (time_now_s - last_gps_time_s > 1.0f) {
    last_gps_time_s = time_now_s;

    _robot.pos.x   = _robot.pos.x + _KF.cov[0][0]/(1.+_KF.cov[0][0])*(_meas.gps[0]-_robot.pos.x);
    _robot.pos.y   = _robot.pos.y + _KF.cov[1][1]/(1.+_KF.cov[1][1])*(_meas.gps[2]-_robot.pos.y);
    _robot.speed.x = _robot.speed.x + _KF.cov[0][2]/(1.+_KF.cov[0][0])*(_meas.gps[0]-_robot.pos.x);
    _robot.speed.y = _robot.speed.y + _KF.cov[1][3]/(1.+_KF.cov[1][1])*(_meas.gps[2]-_robot.pos.y);
    
    _KF.new_cov[0][0]=_KF.cov[0][0] / (1.+_KF.cov[0][0]);
    _KF.new_cov[0][2]=_KF.cov[0][2] / (1.+_KF.cov[0][0]);
    _KF.new_cov[1][1]=_KF.cov[1][1] / (1.+_KF.cov[1][1]);
    _KF.new_cov[1][3]=_KF.cov[1][3] / (1.+_KF.cov[1][1]);
    _KF.new_cov[2][0]=_KF.cov[2][0] - _KF.cov[0][0]*_KF.cov[0][2] / (1.+_KF.cov[0][0]);
    _KF.new_cov[2][2]=_KF.cov[2][2] - _KF.cov[0][2]*_KF.cov[2][0]/(1.+_KF.cov[0][0]);
    _KF.new_cov[3][1]=_KF.cov[3][1] - _KF.cov[1][1]*_KF.cov[1][3] / (1.+_KF.cov[1][1]);
    _KF.new_cov[3][3]=_KF.cov[3][3] - _KF.cov[1][3]*_KF.cov[3][1]/(1.+_KF.cov[2][2]);
    
    for (i=0; i<4; i++){
      for (j=0; j<4; j++) _KF.cov[i][j]=_KF.new_cov[i][j];
    }
  }  
}




int main() 
{

  init_Kalman_Filter();
  
  //print_data();


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

