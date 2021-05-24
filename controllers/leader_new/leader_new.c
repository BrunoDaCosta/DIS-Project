#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>

#include "odometry.h"
#include "utils.h"

#define VERBOSE_GPS false
#define VERBOSE_ENC false

#define VERBOSE_POS false
#define VERBOSE_KF false

#define ACTIVATE_KALMAN true


/*CONSTANTES*/
#define WHEEL_AXIS 	0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020		// Radius of the wheel in meter

#define NB_SENSOR 8
#define MAX_SPEED_WEB      6.28    // Maximum speed webots

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
  float supervisor[3];
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

//int Interconn[16] = {-5,-15,-20,6,4,6,3,5,    4,4,6,-18,-15,-5,5,3};
//Still to modify
double Interconn[16] = {20,10,5,0,0,-4,-9,-19,-20,-10,-5,0,0,4,9,19}; //minus 1 in left side to avoid frontal collision
WbDeviceTag ds[NB_SENSOR];
WbDeviceTag emitter;                 

// TO REMOVE !!!!
WbDeviceTag actual_pos;

//-----------------------------------------------------------------------------------//
/*VARIABLES*/
static measurement_t  _meas;
static robot_t        _robot;

static double KF_cov[MMS][MMS]={{0.001, 0, 0, 0},
                                {0, 0.001, 0, 0},
                                {0, 0, 0.001, 0},
                                {0, 0, 0, 0.001}};
//static motor_t        _motor = {false, true, true, MAX_SPEED / 4.0};
double last_gps_time_s = 0.0f;
double time_end_calibration = 0;

static FILE *fp;

//-----------------------------------------------------------------------------------//


static void controller_get_encoder();
static void controller_get_gps();
static void controller_print_log();
static bool controller_init_log(const char* filename);
static void trajectory();
static void braiten();
static void saturate_velocity();
static double lookuptable_sensor(int value);
double traj_vl;
double traj_vr;

void init_devices(int ts);

void init_devices(int ts) {
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps, 1000);
  
  // TO REMOVE !!!!
  actual_pos = wb_robot_get_device("gps");
  wb_gps_enable(actual_pos, 1);

  dev_acc = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(dev_acc, ts);
  
  emitter = wb_robot_get_device("emitter");

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
  
  char text[4] = "ps0";
  int i;
  for (i=0;i<NB_SENSOR;i++) {
    ds[i] = wb_robot_get_device(text); /* distance sensors */
    text[2]++;
  }
  for(i=0;i<NB_SENSOR;i++) {
    wb_distance_sensor_enable(ds[i],wb_robot_get_basic_time_step());
  }
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

 // if (DEBUG_PRINT)
    //print_matrix(KF_cov, 4,4);

  mult(A,KF_cov,A_cov,4,4,4,4);
  transp(A,AT,4,4);
  mult(A_cov, AT, A_cov_AT, 4,4,4,4);

  scalar_mult(ts, R, ts_R, 4, 4);
  add(A_cov_AT, ts_R, KF_cov, 4,4,4,4);
  
}


void Kalman_Filter(){
  static double X[MMS][MMS];
  X[0][0]=_robot.pos.x;
  X[1][0]=_robot.pos.y;
  X[2][0]=_robot.speed.x;
  X[3][0]=_robot.speed.y;
  
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


  if (VERBOSE_KF){
    printf("Cov matrix \n");
    print_matrix(KF_cov, 4,4);
  
    printf("X matrix \n");
    print_matrix(X_new, 4,1);
  }
}



int main()
{
  
  if (controller_init_log("odoenc.csv")) return 1;
  _robot.pos.x = -2.9;
  _robot.pos.y = 0;
  _robot.pos.heading = 0;
  _robot.speed.x = 0;
  _robot.speed.y = 0;

  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);

  while (wb_robot_step(time_step) != -1)  {
    printf("In while \n");
    
    trajectory(dev_left_motor, dev_right_motor,time_end_calibration);
    traj_vl = traj_vl/2;
    traj_vr = traj_vr/2;
    braiten();
    saturate_velocity();
    
    wb_motor_set_velocity(dev_left_motor, traj_vl);
    wb_motor_set_velocity(dev_right_motor, traj_vr);
    controller_get_encoder();
    
    
    KF_Update_Cov_Matrix((double) time_step/1000);

    double time_now_s = wb_robot_get_time();
    if (ACTIVATE_KALMAN &&   time_now_s - last_gps_time_s >= 1.0f) {
      last_gps_time_s = time_now_s;
      controller_get_gps();
      if (VERBOSE_POS)  printf("ROBOT pose: %g %g %g\n", _robot.pos.x , _robot.pos.y, _robot.pos.heading);
      //printf("ACC1: %g %g %g\n", _robot.acc.x , _robot.acc.y, _robot.acc.heading);
      Kalman_Filter();
      //printf("ACC2: %g %g %g\n", _robot.acc.x , _robot.acc.y, _robot.acc.heading);

      if (VERBOSE_POS)  printf("ROBOT pose after Kalman: %g %g %g\n\n", _robot.pos.x , _robot.pos.y, _robot.pos.heading);
    }
    controller_print_log();

  }
  
  // Close the log file
  if(fp != NULL)
    fclose(fp);
    
   // End of the simulation
  wb_robot_cleanup();

  return 0;
}

void braiten() {
	unsigned int ds_value[NB_SENSOR];
	int i;
	/*for (i=0;i<NB_SENSOR;i++) {
               ds_value[i] = wb_distance_sensor_get_value(ds[i]);
	}
	
	printf("Values: %d, %d, %d, %d, %d, %d, %d, %d \n", ds_value[0], ds_value[1], ds_value[2], ds_value[3], ds_value[4], ds_value[5], ds_value[6], ds_value[7]);
	*/
	
	double newleft = 0;
	double newright = 0;
	/*
	for (i=0;i<NB_SENSOR;i++) {
            	if(ds_value[i]>100 && ds_value[i]<65000){
            		newright += 10*ds_value[i] * Interconn[i];
            		newleft += 10*ds_value[i] * Interconn[i+NB_SENSOR];
              	}
	}*/
	
	for (i=0;i<NB_SENSOR;i++) {
            	if(lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))!=1){
            		newright += 200*(1/lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))) * Interconn[i];
            		newleft += 200*(1/lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))) * Interconn[i+NB_SENSOR];
              	}
	}
	printf("Newright: %f  Newleft: %f \n", newright/400*MAX_SPEED_WEB/1000, newleft/400*MAX_SPEED_WEB/1000);
	printf("Traj_vl_before %f \n",traj_vl);
	printf("Traj_vr_before %f \n",traj_vr);
	traj_vl += newleft/400*MAX_SPEED_WEB/1000;
	traj_vr += newright/400*MAX_SPEED_WEB/1000;
	printf("Traj_vl_after %f \n",traj_vl);
	printf("Traj_vr_after %f \n",traj_vr);
}

double lookuptable_sensor(int value){
  //return 1 if value is strange
  if(value>4095)
    return 1;
    
    
  else if(value>3474)
    return 0.005-(value-3474)/(4095-3474)*0.005; 
  else if(value>2211)
    return 0.01-(value-2211)/(3473-2211)*(0.01-0.005); 
  else if(value>676)
    return 0.02-(value-676)/(2211-676)*(0.02-0.01); 
  else if(value>306)
    return 0.03-(value-306)/(676-306)*(0.03-0.02); 
    
    //return 1 to avoid noise
  else if(value>34)
    return 1;
    //return 0.04-(value-34)/(306-34)*(0.04-0.03); 
  else if(value>=0)
    return 1;
    //return 1-(value-0)/(34-0)*(1-0.04);
  else 
    return 1;
}

void saturate_velocity() {

  if(traj_vl>MAX_SPEED_WEB)
    traj_vl=MAX_SPEED_WEB;
    
  if(traj_vr>MAX_SPEED_WEB)
    traj_vr=MAX_SPEED_WEB;
    
  if(traj_vl<-MAX_SPEED_WEB)
    traj_vl=-MAX_SPEED_WEB;
    
  if(traj_vr<-MAX_SPEED_WEB)
    traj_vr=-MAX_SPEED_WEB;
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

  double omega = - ( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step ); //ADDED MINUS TO TEST --JEREMY
  double speed = ( deltaright + deltaleft ) / ( 2.0 * time_step );

  double a = _robot.pos.heading;

  double speed_wx = speed * cos(a);
  double speed_wy = speed * sin(a);


  //A enlever à terme
  _robot.speed.x = speed_wx;
  _robot.speed.y = speed_wy;
  _robot.pos.x += _robot.speed.x * time_step;
  _robot.pos.y += _robot.speed.y * time_step;
  _robot.pos.heading += omega * time_step;

  if(VERBOSE_ENC)
    printf("ROBOT enc : x: %g  y: %g heading: %g\n", _robot.pos.x, _robot.pos.y, _robot.pos.heading);
}



/**
 * @brief     Get the gps measurements for the position of the robot. Get the heading angle. Fill the pose structure.
 */

void controller_get_gps(){
  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  const double * gps_position = wb_gps_get_values(dev_gps);
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));
}

/**
 * @brief      Log the usefull informations about the simulation
 *
 * @param[in]  time  The time
 */
void controller_print_log()
{
  // TO REMOVE !!!!
  const double * gps_position = wb_gps_get_values(actual_pos);
  
  if( fp != NULL){
    fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
            wb_robot_get_time(), _robot.pos.x, _robot.pos.y , _robot.pos.heading, _meas.gps[0], _meas.gps[2],
             _robot.speed.x, _robot.speed.y, _robot.acc.x, _robot.acc.y, gps_position[0], gps_position[2]);
  }
}

/**
 * @brief      Initialize the logging of the file
 *
 * @param[in]  filename  The filename to write
 *
 * @return     return true if it fails
 */
bool controller_init_log(const char* filename){
  fp = fopen(filename,"w");
  bool err = (fp == NULL);

  if( !err ){
    fprintf(fp, "time; pose_x; pose_y; pose_heading;  gps_x; gps_y; speed_x; speed_y; acc_x; acc_y; actual_pos_x; actual_pos_y\n");
  }
  return err;
}

void trajectory() {
// ## DO NOT MODIFY THIS
   //double t = wb_robot_get_time();
   double t = 1000;
   if (t > 0.0 && t < 3.0) {
     traj_vl = 6.28;
     traj_vr = 6.28;
   }
   
   else if (t >= 3.0 && t < 5.0) { // right turn
     traj_vl = 6.28;
     traj_vr = 4.0;
   }
   
   else if (t >= 5.0 && t < 14.0) {
     traj_vl = 6.28;
     traj_vr = 6.28;
   }
   
   else if (t >= 14.0 && t < 16.0) { // left turn
     traj_vl = 4.0;
     traj_vr = 6.28;
   }
   
   else if (t >= 16.0 && t < 50.0) {
     traj_vl = 6.28;
     traj_vr = 6.28;
   }
   
   else if (t >= 50.0 && t < 52.0) { // left turn
     traj_vl = 4.0;
     traj_vr = 6.28;
   }
   
   else if (t >= 52.0 && t < 70.0) {
     traj_vl = 6.28;
     traj_vr = 6.28;
   }
   
   else if (t >= 70.0 && t < 71.9) { // left turn
     traj_vl = 4.0;
     traj_vr = 6.28;
   }
   
   else if (t >= 72.0 && t < 105.0) {
     traj_vl = 6.28;
     traj_vr = 6.28;
   }
   
   else if (t >= 105.0 && t < 106.9) { // left turn
     traj_vl = 4.0;
     traj_vr = 6.28;
   }
   
   else if (t >= 107.0 && t < 115.0) {
     traj_vl = 6.28;
     traj_vr = 6.28;
   }
    
   else {
     traj_vl = 6.28;
     traj_vr = 6.28;
   }
}