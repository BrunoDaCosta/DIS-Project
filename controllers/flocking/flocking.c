#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "kalman.h"
#include "trajectories.h"
#include "utils.h"

// Verbose activation
#define VERBOSE_GPS false
#define VERBOSE_ENC false
#define VERBOSE_ACC false

#define VERBOSE_POS false
#define VERBOSE_ACC_MEAN false
// #define VERBOSE_KF false

// Odometry type
#define ODOMETRY_ACC false
#define ACTIVATE_KALMAN true


/*CONSTANTES*/
#define WHEEL_AXIS 	0.052 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020	           // Radius of the wheel in meter
#define TIME_INIT_ACC 5                     // Time in second
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second

#define NB_SENSORS    8	           // Number of distance sensors
#define FLOCK_SIZE	5                     // Size of flock


#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED_WEB     6.27    // Maximum speed webots
#define MAX_SPEED         800     // Maximum speed

#define RULE1_THRESHOLD     0.20          // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)      // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.20         // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10) // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)      // Weight of consistency rule. default 1.0/10

#define MIGRATORY_URGE    1
#define MIGRATION_WEIGHT  (0.01/10)*20    // Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATION_DIST    (0.01/10)

#define FACTOR             1.0

#define M_PI 3.14159265358979323846
#define SIGN(x) ((x>=0)?(1):-(1))

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
  int id;
  pose_t pos;
  pose_t prev_pos;
  pose_t speed;
  pose_t acc;
  pose_t rel_pos;
  pose_t rel_prev_pos;
  pose_t rel_speed;
  pose_t rey_speed;
}robot_t;

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
static robot_t rf[FLOCK_SIZE];
static int iter =0;

double last_gps_time_s = 0.0f;
double time_end_calibration = 0;

int Interconn[16] = {20,30,30,10,10,-5,-9,-19,-20,-10,-5,9,9,28,28,19}; // Maze
//int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
//int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
float INITIAL_POS[FLOCK_SIZE][3] = {{-2.9, 0, 0}, {-2.9, 0.1, 0}, {-2.9, -0.1, 0}, {-2.9, 0.2, 0}, {-2.9, -0.2, 0}};

float migr[2] = {3.2, 0};	                // Migration vector

static FILE *fp;

char* robot_name;

//-----------------------------------------------------------------------------------//


static void controller_get_acc();
static void controller_get_encoder();
static void controller_get_gps();
static void controller_compute_mean_acc();
static void controller_print_log();
static bool controller_init_log(const char* filename);

static void odometry_update(int time_step);

static void process_received_ping_messages(int time_step);
static void send_ping();

void init_devices(int ts);

void init_devices(int ts){
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
    robot_name=(char*) wb_robot_get_name();

    for(i=0;i<NB_SENSORS;i++) {
        wb_distance_sensor_enable(ds[i],ts);
    }
    wb_receiver_enable(receiver,ts);

    sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
    robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
    migr[1]=INITIAL_POS[robot_id][1];

    for(i=0; i<FLOCK_SIZE; i++) {
        rf[i].pos.x = INITIAL_POS[i][0];
        rf[i].pos.y = INITIAL_POS[i][1];
        rf[i].pos.heading = INITIAL_POS[i][2];

        rf[i].rel_prev_pos.x = INITIAL_POS[i][0];
        rf[i].rel_prev_pos.y = INITIAL_POS[i][1];
    }

    printf("Init: robot %d\n",robot_id_u);
}





void braitenberg(float* msl, float* msr){
    int i;				// Loop counter
    int bmsl = 0;
    int bmsr = 0;

    for (i=0;i<NB_SENSORS;i++) {
        //if (robot_id ==2) printf("Values detected capteur %d : %f\n", i,wb_distance_sensor_get_value(ds[i]));
        if(lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))!=1){
            bmsr += 200*(1/lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))) * Interconn[i] * FACTOR;
            bmsl += 200*(1/lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))) * Interconn[i+NB_SENSORS] * FACTOR;
        }
    }
    
   
   //if (robot_id == 2) printf("Robot %d, brait: bmsr = %d, bmsl = %d\n",robot_id, bmsr, bmsl);
    
    if (abs(bmsr) > 0 || abs(bmsl) > 0){
      *msl = *msl*0.1 +  bmsl/400*MAX_SPEED_WEB/1000;
      *msr = *msr*0.1 +  bmsr/400*MAX_SPEED_WEB/1000;
    
    }else{
      *msl += bmsl/400*MAX_SPEED_WEB/1000;
      *msr += bmsr/400*MAX_SPEED_WEB/1000;
    }
    
    
    /* // Adapt Braitenberg values (empirical tests)
    if (abs(bmsr) > 0 || abs(bmsl) > 0){
    
      *msl = bmsl/400*MAX_SPEED_WEB/1000;
      *msr = bmsr/400*MAX_SPEED_WEB/1000;
      
      if (robot_id== 2) printf("Avoiding\n");
      iter = 10;
    }else{
      if (iter != 0){
        iter--;
        if (robot_id == 2) printf("Iter avoidance %d",iter);
      }
      
      if (iter == 0){
         *msl += bmsl/400*MAX_SPEED_WEB/1000;
         *msr += bmsr/400*MAX_SPEED_WEB/1000;
      }else{
         *msl = 1;
         *msr = 1;
      }
   }*/ 
}


void reynolds_rules() {
	int i, k;			// Loop counters
	float avg_loc[2] = {0,0};	// Flock average positions
	float avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};

	/* Compute averages over the whole flock */
	for(i=0; i<FLOCK_SIZE; i++) {
		if (i == robot_id) continue; // don't consider yourself for the average
        avg_speed[0] += rf[i].rel_speed.x;
        avg_speed[1] += rf[i].rel_speed.y;
        avg_loc[0] += rf[i].rel_pos.x;
        avg_loc[1] += rf[i].rel_pos.y;

    }
	avg_speed[0] /= FLOCK_SIZE-1;
    avg_speed[1] /= FLOCK_SIZE-1;
	avg_loc[0] /= FLOCK_SIZE-1;
    avg_loc[1] /= FLOCK_SIZE-1;

	/* Reynold's rules */

	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
    if (sqrt(pow(avg_loc[0],2)+pow(avg_loc[1],2)) > RULE1_THRESHOLD){
        cohesion[0] = avg_loc[0];   // Relative distance in x to the center of the swarm
        cohesion[1] = avg_loc[1];   // Relative distance in y to the center of the swarm
    }

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for (k=0;k<FLOCK_SIZE;k++) {
  	   if (k != robot_id) {        // Loop on flockmates only
	      // If neighbor k is too close (Euclidean distance)
	      if (sqrt(pow(rf[k].rel_pos.x,2)+pow(rf[k].rel_pos.y,2)) < RULE2_THRESHOLD) {
  	         float value = 1/sqrt(pow(rf[k].rel_pos.x,2)+pow(rf[k].rel_pos.y,2));
  	         float angle = atan2(rf[k].rel_pos.y,rf[k].rel_pos.x);
  	         dispersion[0] += value*cos(angle);
  	         dispersion[1] -= value*sin(angle);
  	       }
               }
	}
	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */

	consistency[0] = avg_speed[0];
	consistency[1] = avg_speed[1];

    rf[robot_id].rey_speed.x = cohesion[0]*RULE1_WEIGHT + dispersion[0]*RULE2_WEIGHT + consistency[0]*RULE3_WEIGHT;
    rf[robot_id].rey_speed.y = cohesion[1]*RULE1_WEIGHT + dispersion[1]*RULE2_WEIGHT + consistency[1]*RULE3_WEIGHT;

	//move the robot according to some migration rule
	if(MIGRATORY_URGE == 0){
		rf[robot_id].rey_speed.x += 0*0.01*cos(rf[robot_id].pos.heading + M_PI/2);
		rf[robot_id].rey_speed.y += 0*0.01*sin(rf[robot_id].pos.heading + M_PI/2);
	} else {
		/* Implement migratory urge */
              if(fabs(migr[0]-rf[robot_id].pos.x)>500*MIGRATION_DIST){
                rf[robot_id].rey_speed.x += MIGRATION_WEIGHT*SIGN(migr[0]-rf[robot_id].pos.x);
                 /* if(fabs(migr[1]-rf[robot_id].pos.y)>500* MIGRATION_DIST){
                     rf[robot_id].rey_speed.y += MIGRATION_WEIGHT*SIGN(migr[1]-rf[robot_id].pos.y);
                 }*/
               }
            else if(fabs(migr[0]-rf[robot_id].pos.x)>MIGRATION_DIST || fabs(migr[1]-rf[robot_id].pos.y)>MIGRATION_DIST){

                if(fabs(migr[0]-rf[robot_id].pos.x)>MIGRATION_DIST)
                {
                  rf[robot_id].rey_speed.x += MIGRATION_WEIGHT*SIGN(migr[0]-rf[robot_id].pos.x);
                  }
                  if(fabs(migr[1]-rf[robot_id].pos.y)>MIGRATION_DIST)
                  {
                     rf[robot_id].rey_speed.y += MIGRATION_WEIGHT*SIGN(migr[1]-rf[robot_id].pos.y);
                 }
                }
    	}
    if(robot_id==5/*pour pas que ça print*/)
    {
    printf(" \n");
    printf("Robot %d\n",robot_id);
    printf("    X Cohesion: %f, dispersion: %f, consistency: %f, migration: %f, speed: %f \n", cohesion[0]*RULE1_WEIGHT,dispersion[0]*RULE2_WEIGHT,consistency[0]*RULE3_WEIGHT, MIGRATION_WEIGHT*SIGN(migr[0]-rf[robot_id].pos.x),  rf[robot_id].rey_speed.x);
    printf("    Y Cohesion: %f, dispersion: %f, consistency: %f, migration: %f, speed: %f \n", cohesion[1]*RULE1_WEIGHT,dispersion[1]*RULE2_WEIGHT,consistency[1]*RULE3_WEIGHT, 0.0, rf[robot_id].rey_speed.y);
    }
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(float *msl, float *msr)
{
	// Compute wanted position from Reynold's speed and current location
	float Ku = 0.4;   // Forward control coefficient
	float Kw = 0.3;  // Rotational control coefficient
	float range = sqrtf(rf[robot_id].rey_speed.x*rf[robot_id].rey_speed.x +rf[robot_id].rey_speed.y*rf[robot_id].rey_speed.y);	  // Distance to the wanted position
	float bearing = atan2(rf[robot_id].rey_speed.y, rf[robot_id].rey_speed.x);	  // Orientation of the wanted position

	float delta = bearing-rf[robot_id].pos.heading;
	if(delta>M_PI)
                delta-=2*M_PI;
	if(delta<-M_PI)
                delta+=2*M_PI;

           // Compute forward control
	float u = Ku*range*cosf(delta);

	// Compute rotational control
	float w = Kw*(delta);
	// Convert to wheel speeds!
	*msl = (u + WHEEL_AXIS*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u - WHEEL_AXIS*w/2.0) * (1000.0 / WHEEL_RADIUS);

	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);

    *msl = ((float) *msl)*MAX_SPEED_WEB/MAX_SPEED;
    *msr = ((float) *msr)*MAX_SPEED_WEB/MAX_SPEED;
}



int main()
{
  float msl, msr;
  

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

  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);


  send_ping();
  while (wb_robot_step(time_step) != -1)  {

    send_ping();
    process_received_ping_messages(time_step);

    odometry_update(time_step);
    controller_print_log();

    // Reynold's rules with all previous info (updates the speed[][] table)
    reynolds_rules();

    if(robot_id==5) printf(" \n");
    // Compute wheels speed from Reynold's speed
    compute_wheel_speeds(&msl, &msr);
    // if(robot_id==2) printf("Before msl: %f msr: %f\n",msl,msr);
    // Add Braitenberg
    braitenberg(&msl, &msr);
    //if(robot_id==2) printf("After msl: %f msr: %f\n",msl,msr);

    limit(&msl, 6.27);
    limit(&msr, 6.27);
    wb_motor_set_velocity(dev_left_motor, msl);
    wb_motor_set_velocity(dev_right_motor, msr);
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
    if (VERBOSE_POS)  printf("ROBOT %d pose:\t %g %g %g | GPS data:%g %g\n", robot_id, rf[robot_id].pos.x , rf[robot_id].pos.y, rf[robot_id].pos.heading, _meas.gps[0], _meas.gps[2]);
    //printf("ACC1: %g %g %g\n", rf[robot_id].acc.x , rf[robot_id].acc.y, rf[robot_id].acc.heading);
    //if (robot_id==0) printf("GPS data: %g %g\n", _meas.gps[0], _meas.gps[1]);
    Kalman_Filter(&rf[robot_id].pos.x , &rf[robot_id].pos.y, &rf[robot_id].pos.heading, &rf[robot_id].speed.x, &rf[robot_id].speed.y, &_meas.gps[0], &_meas.gps[2]);
    //print_cov_matrix();
    //printf("ACC2: %g %g %g\n", rf[robot_id].acc.x , rf[robot_id].acc.y, rf[robot_id].acc.heading);
    if (VERBOSE_POS)  printf("ROBOT pose after Kalman: %g %g %g\n\n", rf[robot_id].pos.x , rf[robot_id].pos.y, rf[robot_id].pos.heading);
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

    double a = rf[robot_id].pos.heading;

    double speed_wx = speed * cos(a);
    double speed_wy = speed * sin(a);



    rf[robot_id].speed.x = speed_wx;
    rf[robot_id].speed.y = speed_wy;
    rf[robot_id].pos.x += rf[robot_id].speed.x * time_step;
    rf[robot_id].pos.y += rf[robot_id].speed.y * time_step;
    rf[robot_id].pos.heading += omega * time_step;
    if (rf[robot_id].pos.heading>M_PI) rf[robot_id].pos.heading-=2*M_PI;
    if (rf[robot_id].pos.heading<-M_PI) rf[robot_id].pos.heading+=2*M_PI;

    //if(robot_id==4) printf("In odometry: %g %g\n", _meas.left_enc, _meas.right_enc);
    if(VERBOSE_ENC)
        printf("ROBOT enc : x: %g  y: %g heading: %g\n", rf[robot_id].pos.x, rf[robot_id].pos.y, rf[robot_id].pos.heading);
}

void controller_get_acc()
{
  int time_step = wb_robot_get_basic_time_step();

  const double * acc_values = wb_accelerometer_get_values(dev_acc);
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  double accfront = ( _meas.acc[1] - _meas.acc_mean[1]);
  //double accside = ( _meas.acc[0] - _meas.acc_mean[0]);
  double heading_tmp = rf[robot_id].pos.heading;

  _meas.prev_left_enc = _meas.left_enc;
  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  double deltaleft=_meas.left_enc-_meas.prev_left_enc;
  _meas.prev_right_enc = _meas.right_enc;
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
  double deltaright=_meas.right_enc-_meas.prev_right_enc;
  deltaleft  *= WHEEL_RADIUS;
  deltaright *= WHEEL_RADIUS;
  double omega = -( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step );
  rf[robot_id].pos.heading += omega * time_step;
  if (rf[robot_id].pos.heading>M_PI) rf[robot_id].pos.heading-=2*M_PI;
  if (rf[robot_id].pos.heading<-M_PI) rf[robot_id].pos.heading+=2*M_PI;

  double delta_heading = rf[robot_id].pos.heading - heading_tmp;

  rf[robot_id].acc.x= accfront * cos(rf[robot_id].pos.heading);
  rf[robot_id].acc.y= accfront * sin(rf[robot_id].pos.heading);

  double spxtmp = rf[robot_id].speed.x;
  double spytmp = rf[robot_id].speed.y;
  rf[robot_id].speed.x = cos(delta_heading)*spxtmp - sin(delta_heading)*spytmp + rf[robot_id].acc.x * time_step/ 1000.0;
  rf[robot_id].speed.y = sin(delta_heading)*spxtmp + cos(delta_heading)*spytmp + rf[robot_id].acc.y * time_step/ 1000.0;
  rf[robot_id].pos.x += rf[robot_id].speed.x * time_step/ 1000.0;
  rf[robot_id].pos.y += rf[robot_id].speed.y * time_step/ 1000.0;

  if(VERBOSE_ACC)
    printf("ROBOT acc : x: %g  y: %g heading: %g\n", rf[robot_id].pos.x, rf[robot_id].pos.y, rf[robot_id].pos.heading);


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
            wb_robot_get_time(), rf[robot_id].pos.x, rf[robot_id].pos.y , rf[robot_id].pos.heading, _meas.gps[0], _meas.gps[2],
             rf[robot_id].speed.x, rf[robot_id].speed.y, rf[robot_id].acc.x, rf[robot_id].acc.y);
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


 /*
  *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
  *  the message contains the robot's name
  *  the range and bearing will be measured directly out of message RSSI and direction
 */
 void send_ping() {
 	char out[10];
 	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
 	wb_emitter_send(emitter,out,strlen(out)+1);
 }

 /*
  * processing all the received ping messages, and calculate range and bearing to the other robots
  * the range and bearing are measured directly out of message RSSI and direction
 */
 void process_received_ping_messages(int time_step) {
 	const double *message_direction;
 	double message_rssi; // Received Signal Strength indicator
 	double theta;
 	double range;
 	char *inbuffer;	// Buffer for the receiver node
 	int other_robot_id;
    int iter = 0;
 	while (wb_receiver_get_queue_length(receiver) > 0) {
        iter+=1;
 		inbuffer = (char*) wb_receiver_get_data(receiver);
 		message_direction = wb_receiver_get_emitter_direction(receiver);
 		message_rssi = wb_receiver_get_signal_strength(receiver);

 		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

                      //theta = message_direction[0]*M_PI/2; //+ rf[robot_id].pos.heading;
                      // printf("Theta from message is %f\n", theta);
                      //double y = message_direction[2];
                      // double x = message_direction[1];


                      //theta = -atan2(y,x) + rf[robot_id].pos.heading;
                      //if(robot_id==4)printf("Rob %d from rob %d 0: %f, 1: %f, 2: %f \n", robot_id,other_robot_id,message_direction[0],message_direction[1],message_direction[2]);
                      //theta = message_direction[0]*M_PI/2 + rf[robot_id].pos.heading;
                      double y=message_direction[0];
                      double x=-message_direction[2];
                      theta = atan2(y,x) + rf[robot_id].pos.heading;
                      //if(robot_id==4)printf("Rob %d from rob %d Theta: %f\n", robot_id,other_robot_id,theta);

                      //printf("Theta = %f\n", theta);
 		range = sqrt((1/message_rssi));

                      //printf("Robot %d from %d, x = %f, y = %f, dim_3 = %f, my_theta = %f, theta = %f\n", robot_id, other_robot_id, x, y, message_direction[0], 270 - rf[robot_id].pos.heading*180.0/3.141592, theta*180.0/3.141592);

 		// Get position update
 		rf[other_robot_id].rel_prev_pos.x = rf[other_robot_id].rel_pos.x;
 		rf[other_robot_id].rel_prev_pos.y = rf[other_robot_id].rel_pos.y;

 		rf[other_robot_id].rel_pos.x = range*cos(theta);  // relative x pos
 		rf[other_robot_id].rel_pos.y = range*sin(theta);   // relative y pos
 		//if(robot_id==4)printf("Rob %d from rob %d X: %f Y: %f\n", robot_id,other_robot_id,rf[other_robot_id].rel_pos.x,rf[other_robot_id].rel_pos.y);

 		//if(robot_id==4)printf("Rob %d from rob %d X: %f Y: %f\n", robot_id,other_robot_id,rf[other_robot_id].rel_pos.x,rf[other_robot_id].rel_pos.y);

 		//printf("Robot %d from %d, rel_x = %f, rel_y = %f\n", robot_id, other_robot_id, rf[other_robot_id].rel_pos.x, rf[other_robot_id].rel_pos.y);

        rf[other_robot_id].rel_speed.x = (1/((float) time_step))*(rf[other_robot_id].rel_pos.x-rf[other_robot_id].rel_prev_pos.x);
        rf[other_robot_id].rel_speed.y = (1/((float) time_step))*(rf[other_robot_id].rel_pos.y-rf[other_robot_id].rel_prev_pos.y);

        //printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, sp x:%g, sp y:%g, queue length: %d\n",robot_name, other_robot_id,rf[other_robot_id].rel_pos.x,rf[other_robot_id].rel_pos.y,theta,rf[other_robot_id].rel_speed.x, rf[other_robot_id].rel_speed.y, wb_receiver_get_queue_length(receiver));
        //printf("    0: %f 1: %f 2: %f %f\n", message_direction[0], message_direction[1], message_direction[2], -atan2(-message_direction[2],message_direction[0]));
        wb_receiver_next_packet(receiver);
 	}
 }
