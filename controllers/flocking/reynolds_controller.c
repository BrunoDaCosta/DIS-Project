#include <stdio.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "trajectories.h"
#include "utils.h"



// Verbose activation
#define VERBOSE_GPS false
#define VERBOSE_ENC false
#define VERBOSE_ACC false

#define VERBOSE_POS false
#define VERBOSE_ACC_MEAN false
#define VERBOSE_KF false

// Odometry type
#define ODOMETRY_ACC false
#define ACTIVATE_KALMAN true


/*CONSTANTES*/
#define WHEEL_AXIS 	0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020	           // Radius of the wheel in meter
#define TIME_INIT_ACC 5                     // Time in second

#define NB_SENSORS    8	           // Number of distance sensors
#define FLOCK_SIZE	5                     // Size of flock

#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value



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
WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

static int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

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
double time_end_calibration = 0;

int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze

static FILE *fp;

//-----------------------------------------------------------------------------------//


static void controller_get_acc();
static void controller_get_encoder();
static void controller_get_gps();
static void controller_compute_mean_acc();
static void controller_print_log();
static bool controller_init_log(const char* filename);

static void odometry_update(int time_step);

void init_devices(int ts);

void init_devices(int ts) {
  // GPS
  dev_gps = wb_robot_get_device("gps");
  wb_gps_enable(dev_gps, 1000);
  // Accelerometer
  dev_acc = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(dev_acc, ts);
  // Encoder
  dev_left_encoder = wb_robot_get_device("left wheel sensor");
  dev_right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(dev_left_encoder,  ts);
  wb_position_sensor_enable(dev_right_encoder, ts);

  // Motor control
  dev_left_motor = wb_robot_get_device("left wheel motor");
  dev_right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(dev_left_motor, INFINITY);
  wb_motor_set_position(dev_right_motor, INFINITY);
  wb_motor_set_velocity(dev_left_motor, 0.0);
  wb_motor_set_velocity(dev_right_motor, 0.0);
  
  // Communication 
  receiver = wb_robot_get_device("receiver");
  emitter = wb_robot_get_device("emitter");
  int i;
  char s[4]="ps0";
  for(i=0; i<NB_SENSORS;i++) {
    ds[i]=wb_robot_get_device(s); // the device name is specified in the world file
    s[2]++;			 // increases the device number
  }
  char* robot_name; 
  robot_name=(char*) wb_robot_get_name(); 

  for(i=0;i<NB_SENSORS;i++) {
    wb_distance_sensor_enable(ds[i],64);
  }
  wb_receiver_enable(receiver,64);
  
  sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
  robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
  
  // for(i=0; i<FLOCK_SIZE; i++) {
    // initialized[i] = 1; 		  // Set initialization to 0 (= not yet initialized)
  // }
  
  printf("Init: robot %d\n",robot_id_u);
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
    printf("After\n");
    printf("Cov matrix\n");
    print_matrix(KF_cov, 4,4);
  
    printf("X matrix\n");
    print_matrix(X_new, 4,1);
  }
}

void braitenberg(){
  int bmsl = 0, bmsr = 0, sum_sensors = 0;	// Braitenberg parameters
  int i;				// Loop counter
  int distances[NB_SENSORS];	// Array for the distance sensor readings
  int max_sens = 0;			// Store highest sensor value

  /* Braitenberg */
  for(i=0;i<NB_SENSORS;i++) {
    distances[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
    sum_sensors += distances[i]; // Add up sensor values
    max_sens = max_sens>distances[i]?max_sens:distances[i]; // Check if new highest sensor value

    // Weighted sum of distance sensor values for Braitenberg vehicle
    bmsr += e_puck_matrix[i] * distances[i];
    bmsl += e_puck_matrix[i+NB_SENSORS] * distances[i];
  }

  // Adapt Braitenberg values (empirical tests)
  bmsl/=MIN_SENS; bmsr/=MIN_SENS;
  bmsl+=66; bmsr+=72;
}



int main()
{
  /*int rob_nb;			// Robot number
  int msl, msr;			// Wheel speeds
  float msl_w, msr_w;
  char *inbuffer;*/
  char outbuffer[255];			// Buffer for the receiver node
	
  if(ODOMETRY_ACC)
  {
    if (controller_init_log("odoacc.csv")) return 1;
    printf("Use of odometry with accelerometers \n");
  }
  else
  {
    if (controller_init_log("odoenc.csv")) return 1;
    printf("Use of odometry with encoders \n");
  }
  _robot.pos.x = -2.9;
  _robot.pos.y = 0;
  _robot.pos.heading = 0;
  _robot.speed.x = 0;
  _robot.speed.y = 0;

  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);

  while (wb_robot_step(time_step) != -1)  {
  
    odometry_update(time_step);
    
    //_robot.supervisor.y = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
    //_robot.supervisor.heading = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
    controller_print_log();
    
    
    wb_emitter_send(emitter,outbuffer,strlen(outbuffer));
  // Use one of the two trajectories.
    trajectory_1(dev_left_motor, dev_right_motor,time_end_calibration);
//    trajectory_2(dev_left_motor, dev_right_motor,time_end_calibration);
    //wb_robot_step(TIME_STEP);
  }
  
  // Close the log file
  if(fp != NULL)
    fclose(fp);
    
   // End of the simulation
  wb_robot_cleanup();

  return 0;
}


void odometry_update(int time_step){
  if(ODOMETRY_ACC){
    if(wb_robot_get_time() < TIME_INIT_ACC){
      controller_compute_mean_acc();
      time_end_calibration = wb_robot_get_time();
      return; // Maybe return 0 and skip the rest ???
    }
    else
    controller_get_acc();
  }
  else{
    controller_get_encoder();  
  }
  
  KF_Update_Cov_Matrix((double) time_step/1000);

  double time_now_s = wb_robot_get_time();
  if (ACTIVATE_KALMAN &&   time_now_s - last_gps_time_s >= 1.0f){
    last_gps_time_s = time_now_s;
    controller_get_gps();
    if (VERBOSE_POS)  printf("ROBOT pose: %g %g %g\n", _robot.pos.x , _robot.pos.y, _robot.pos.heading);
    //printf("ACC1: %g %g %g\n", _robot.acc.x , _robot.acc.y, _robot.acc.heading);
    Kalman_Filter();
    //printf("ACC2: %g %g %g\n", _robot.acc.x , _robot.acc.y, _robot.acc.heading);
    if (VERBOSE_POS)  printf("ROBOT pose after Kalman: %g %g %g\n\n", _robot.pos.x , _robot.pos.y, _robot.pos.heading);
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

void controller_get_acc()
{
  int time_step = wb_robot_get_basic_time_step();

  const double * acc_values = wb_accelerometer_get_values(dev_acc);
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  double accfront = ( _meas.acc[1] - _meas.acc_mean[1]);
  //double accside = ( _meas.acc[0] - _meas.acc_mean[0]);
  

  double heading_tmp = _robot.pos.heading;
  ///////HEADING/////////
  _meas.prev_left_enc = _meas.left_enc;
  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  double deltaleft=_meas.left_enc-_meas.prev_left_enc;
  _meas.prev_right_enc = _meas.right_enc;
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
  double deltaright=_meas.right_enc-_meas.prev_right_enc;
  deltaleft  *= WHEEL_RADIUS;
  deltaright *= WHEEL_RADIUS;
  double omega = ( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step );
  _robot.pos.heading += omega * time_step;
  ///////////////////////

  double delta_heading = _robot.pos.heading - heading_tmp;

  _robot.acc.x= accfront * cos(_robot.pos.heading);
  _robot.acc.y= accfront * sin(_robot.pos.heading);

  double spxtmp = _robot.speed.x;
  double spytmp = _robot.speed.y;
  _robot.speed.x = cos(delta_heading)*spxtmp - sin(delta_heading)*spytmp + _robot.acc.x * time_step/ 1000.0;
  _robot.speed.y = sin(delta_heading)*spxtmp + cos(delta_heading)*spytmp + _robot.acc.y * time_step/ 1000.0;
  _robot.pos.x += _robot.speed.x * time_step/ 1000.0;
  _robot.pos.y += _robot.speed.y * time_step/ 1000.0;

  if(VERBOSE_ACC)
    printf("ROBOT acc : x: %g  y: %g heading: %g\n", _robot.pos.x, _robot.pos.y, _robot.pos.heading);


}

void controller_compute_mean_acc()
{
  static int count = 0;

  count++;
  const double * acc_values = wb_accelerometer_get_values(dev_acc);
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  if( count > 20 ) // Remove the effects of strong acceleration at the begining
  {
    for(int i = 0; i < 3; i++)
        _meas.acc_mean[i] = (_meas.acc_mean[i] * (count - 21) + _meas.acc[i]) / ((double) count-20);
  }
  int time_step = wb_robot_get_basic_time_step();
  if( count == (int) (TIME_INIT_ACC / (double) time_step) )
    printf("Accelerometer initialization Done ! \n");

  if(VERBOSE_ACC_MEAN)
        printf("ROBOT acc mean : %g %g %g\n", _meas.acc_mean[0], _meas.acc_mean[1] , _meas.acc_mean[2]);
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
  if( fp != NULL){
    fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
            wb_robot_get_time(), _robot.pos.x, _robot.pos.y , _robot.pos.heading, _meas.gps[0], _meas.gps[2],
             _robot.speed.x, _robot.speed.y, _robot.acc.x, _robot.acc.y);
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