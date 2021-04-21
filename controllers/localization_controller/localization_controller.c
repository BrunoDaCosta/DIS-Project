#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

#include "trajectories.h"
#include "odometry.h"


#define KF_NBR_IN 4
#define KF_NBR_OUT 2

#define KF_SIZE 5
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
  //  printf("ROBOT gps is at position: %g %g %g\n", _meas.gps[0], _meas.gps[1], _meas.gps[2]);
  
  _robot.pos.x = _meas.gps[0];
  _robot.pos.y = _meas.gps[2];
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

void mult(double a[][KF_SIZE],double b[][KF_SIZE],double c[][KF_SIZE],int r1,int c1,int r2,int c2){
    int i,j,k;

    for(i=0; i<r1; i++)
        for(j=0; j<c2; j++){
            c[i][j]=0;
        }

    for(i=0; i<r1; i++)
        for(j=0; j<c2; j++)
            for(k=0; k<c1; k++){
                c[i][j]+=a[i][k]*b[k][j];
            }
}



void KF(){

  
  int i,j;
  
  //double K[KF_NBR_IN][KF_NBR_OUT];
  double K[KF_SIZE][KF_SIZE];
  
  double C[KF_SIZE][KF_SIZE] = {{1, 0, 0, 0},
                                {0, 1, 0, 0}};
  double Q[KF_SIZE][KF_SIZE] = {{1, 0},
                                {0, 1}};
                 
  
  
  mult(Q, C, K, KF_NBR_OUT, KF_NBR_IN, KF_NBR_IN, KF_NBR_IN);
  for (i=0; i<KF_NBR_OUT; i++) {
    for (j=0; j<KF_NBR_IN; j++){
      printf("%g ", K[i][j]);
    }
    printf("\n");
  }
                
  
  
  /*
      z = [gpsx(i), gpsy(i)]';
      K = Cov_new * C' * inv(C*Cov_new*C' + Q);
      X_new = X_new + K*(z-C*X_new);
      Cov_new = (eye(4) - K*C)*Cov_new;*/
  
}


int main() 
{
  //KF();

  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);
 
  
  while (wb_robot_step(time_step) != -1)  {
    controller_get_acc();

    controller_get_encoder();
    
    
    
  // Use one of the two trajectories.
    trajectory_1(dev_left_motor, dev_right_motor);
//    trajectory_2(dev_left_motor, dev_right_motor); 
    controller_get_pose();
    if(VERBOSE_GPS)
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

