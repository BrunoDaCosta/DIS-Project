/*****************************************************************************/
/* File:         flocking_supper.c                                           */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  Reynolds flocking control 				     */
/*                                                                           */
/* Author: 	 10-Oct-14 by Ali marjovi				     */
/* Last revision:12-Oct-15 by Florian Maushart				     */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>
#include <webots/differential_wheels.h> // Needed ???
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>


#include "kalman.h"
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
#define ACTIVATE_KALMAN false

#define NB_SENSORS	  8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	  5	  // Size of flock
#define TIME_STEP	  64	  // [ms] Length of time step

#define WHEEL_AXIS 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)
#define TIME_INIT_ACC 5                     // Time in second

#define RULE1_THRESHOLD     0.20   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction

#define M_PI 3.14159265358979323846

#define ABS(x) ((x>=0)?(x):-(x))


typedef struct{
  double prev_gps[3];
  double gps[3];
  double acc_mean[3];
  double acc[3];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;

} measurement_t;
static measurement_t  _meas;

typedef struct{
  float supervisor[3];
  int id;
  pose_t pos;
  pose_t speed;
  pose_t acc;
  pose_t rel_pos;
  pose_t rel_speed;
  pose_t rey_speed;
}robot_t;


WbDeviceTag dev_gps;
WbDeviceTag dev_acc;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;
WbDeviceTag dev_left_motor; //handler for left wheel of the robot
WbDeviceTag dev_right_motor; //handler for the right wheel of the robot

//int Interconn[16] = {20,10,5,20,20,-4,-9,-19,-20,-10,-5,20,20,4,9,19};; // Maze
int Interconn[16] = {17,29,12,10,8,-38,-56,-76,-72,-58,-36,8,10,12,28,18}; // Crossing

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

double loc[FLOCK_SIZE][3];	// X, Z, Theta of all robots
float prev_loc[FLOCK_SIZE][3];	// Previous X, Z, Theta values
double speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
double acc[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];	// != 0 if initial positions have been received
float migr[2];	                // Migration vector

double last_gps_time_s = 0.0f;

float INITIAL_POS[FLOCK_SIZE][3] = {{-2.9, 0, 0}, {-2.9, 0.1, 0}, {-2.9, -0.1, 0}, {-2.9, 0.2, 0}, {-2.9, -0.2, 0}};

static FILE *fp;


static void controller_get_acc();
static void controller_get_encoder();
static void controller_get_gps();
static void controller_compute_mean_acc();
static void controller_print_log();
static bool controller_init_log(const char* filename);

static void odometry_update(int time_step);

void get_data();

void init_devices(int ts);

/*
 * Reset the robot's devices and get its ID
 *
 */
static void reset(int ts) {
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

	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");

	/*Webots 2018b*/
	//get motors
	dev_left_motor = wb_robot_get_device("left wheel motor");
    dev_right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(dev_left_motor, INFINITY);
    wb_motor_set_position(dev_right_motor, INFINITY);
    wb_motor_set_velocity(dev_left_motor, 0.0);
    wb_motor_set_velocity(dev_right_motor, 0.0);
         /*Webots 2018b*/

	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	char* robot_name;
	robot_name=(char*) wb_robot_get_name();

	for(i=0;i<NB_SENSORS;i++) {
		wb_distance_sensor_enable(ds[i],64);
	}
	wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0; 		  // Set initialization to 0 (= not yet initialized)
	}

  	printf("Reset: robot %d\n",robot_id_u);
}

/*
 * Keep given float number within interval {-limit, limit}
 */
void limitf(float *number, int limit) {
	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Updates robot position with wheel speeds
 * Used for odometry
 */
void update_self_motion(int msl, int msr) {
	float theta = loc[robot_id][2];

	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/WHEEL_AXIS;

	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);

	// Update position
	loc[robot_id][0] += dx;
	loc[robot_id][1] += dz;
	loc[robot_id][2] += dtheta;

	// Keep orientation within 0, 2pi
	if (loc[robot_id][2] > 2*M_PI) loc[robot_id][2] -= 2.0*M_PI;
	if (loc[robot_id][2] < 0) loc[robot_id][2] += 2.0*M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(float *msl, float *msr)
{
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(loc[robot_id][2]) + speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(loc[robot_id][2]) + speed[robot_id][1]*cosf(loc[robot_id][2]); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position

	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;

	// Convert to wheel speeds!
	*msl = (u - WHEEL_AXIS*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + WHEEL_AXIS*w/2.0) * (1000.0 / WHEEL_RADIUS);

	limitf(msl,MAX_SPEED);
	limitf(msr,MAX_SPEED);
}


/*
 * Update speed according to Reynold's rules
 */

void reynolds_rules() {
	int i, j, k;			// Loop counters
	float avg_loc[2] = {0,0};	// Flock average positions
	float avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};

	/* Compute averages over the whole flock */
	for(i=0; i<FLOCK_SIZE; i++) {
		if (i == robot_id) {
			// don't consider yourself for the average
			continue;
		}
          	for (j=0;j<2;j++) {
			avg_speed[j] += speed[i][j];
			avg_loc[j] += loc[i][j];
		}
	}

	for (j=0;j<2;j++) {
		avg_speed[j] /= FLOCK_SIZE-1;
		avg_loc[j] /= FLOCK_SIZE-1;
	}
	/* Reynold's rules */

	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	for (j=0;j<2;j++) {
		// If center of mass is too far
		if (sqrt(pow(loc[robot_id][0]-avg_loc[0],2)+pow(loc[robot_id][1]-avg_loc[1],2)) > RULE1_THRESHOLD) {
         		cohesion[j] = avg_loc[j] - loc[robot_id][j];   // Relative distance to the center of the swarm
		}
	}



	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for (k=0;k<FLOCK_SIZE;k++) {
		if (k != robot_id) {        // Loop on flockmates only
			// If neighbor k is too close (Euclidean distance)
			if (pow(loc[robot_id][0]-loc[k][0],2)+pow(loc[robot_id][1]-loc[k][1],2) < RULE2_THRESHOLD) {
				for (j=0;j<2;j++) {
					dispersion[j] += 1/(loc[robot_id][j] -loc[k][j]);	// Relative distance to k
				}
			}
		}
	}

	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
	consistency[0] = 0;
	consistency[1] = 0;
	/* add code for consistency[j]*/
	for (j=0;j<2;j++) {
		consistency[j] = avg_speed[j] - speed[robot_id][j];
	}

	// aggregation of all behaviors with relative influence determined by weights
           // printf("id = %d, cx = %f, cy = %f\n", robot_id, cohesion[0], cohesion[1]);

	if(robot_id == 3) // print only for 1 robot
	printf("id = %d, %f %f, %f %f, %f %f\n", robot_id, cohesion[0], -cohesion[1], dispersion[0], -dispersion[1], consistency[0], -consistency[1]);

	for (j=0;j<2;j++) {
		speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
		speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
		speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT;
	}
	speed[robot_id][1] *= -1; //y axis of webots is inverted

	//move the robot according to some migration rule
	if(MIGRATORY_URGE == 0){
		speed[robot_id][0] += 0*0.01*cos(loc[robot_id][2] + M_PI/2);
		speed[robot_id][1] += 0*0.01*sin(loc[robot_id][2] + M_PI/2);
	} else {
		/* Implement migratory urge */
		speed[robot_id][0] += MIGRATION_WEIGHT*(migr[0]-loc[robot_id][0]);
                      speed[robot_id][1] -= MIGRATION_WEIGHT*(migr[1]-loc[robot_id][1]); //y axis of webots is inverted
	}
}

/*
 * Initialize robot's position
 */
void initial_pos(void){
    loc[robot_id][0] = INITIAL_POS[robot_id][0]; 		// x-position
    loc[robot_id][1] = INITIAL_POS[robot_id][1]; 		// z-position
    loc[robot_id][2] = INITIAL_POS[robot_id][2]; 		// theta

    printf("Initial heading: %g \n", loc[robot_id][2]);
    prev_loc[robot_id][0] = loc[robot_id][0];
    prev_loc[robot_id][1] = loc[robot_id][1];
    initialized[robot_id] = 1;
}

void braitenberg(float* msl, float* msr){
    int i;				// Loop counter
    float factor = 10;
    float bmsl=0, bmsr=0;
    int distances[NB_SENSORS];	// Array for the distance sensor readings
    /* Braitenberg */
    for (i=0;i<NB_SENSORS;i++){
        distances[i]=wb_distance_sensor_get_value(ds[i]);
        if(lookuptable_sensor(distances[i])!=1){
            bmsr += 200*(1/lookuptable_sensor(distances[i])) * Interconn[i] * factor;
            bmsl += 200*(1/lookuptable_sensor(distances[i])) * Interconn[i+NB_SENSORS] * factor;
        }
    }

    if (robot_id==4) printf("Before Braitenberg: %g %g\n", *msl, *msr);

    *msl += bmsl/400*MAX_SPEED_WEB/1000;
    *msr += bmsr/400*MAX_SPEED_WEB/1000;

    if (robot_id==4) printf("After Braitenberg: %g %g %g %g\n", *msl, *msr,bmsl, bmsr);

}


/*
 * Main function
 */
int main(){

	float msl, msr;			// Wheel speeds
	float msl_w, msr_w;
	//int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
	int i;				// Loop counter
	int rob_nb;			// Robot number
	float rob_x, rob_z, rob_theta;  // Robot position and orientation
	int distances[NB_SENSORS];	// Array for the distance sensor readings
	char *inbuffer;
	char outbuffer[255];			// Buffer for the receiver node
	int max_sens;			// Store highest sensor value

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
 	reset(time_step);			// Resetting the robot
	initial_pos();			// Initializing the robot's position

	msl = 0; msr = 0;
	max_sens = 0;

	// Forever
	for(;;){
		/* Get information */
		int count = 0;
		while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE)
		{
			inbuffer = (char*) wb_receiver_get_data(receiver);
			sscanf(inbuffer,"%d#%f#%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta);

			if ((int) rob_nb/FLOCK_SIZE == (int) robot_id/FLOCK_SIZE) {
			rob_nb %= FLOCK_SIZE;
			if (initialized[rob_nb] == 0) {
				// Get initial positions
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_z; //z-position
				loc[rob_nb][2] = rob_theta; //theta
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				initialized[rob_nb] = 1;
			} else {
				// Get position update
//				printf("\n got update robot[%d] = (%f,%f) \n",rob_nb,loc[rob_nb][0],loc[rob_nb][1]);
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_z; //z-position
				loc[rob_nb][2] = rob_theta; //theta
			}

			speed[rob_nb][0] = (1/DELTA_T)*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
			speed[rob_nb][1] = (1/DELTA_T)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
			count++;
			}

			wb_receiver_next_packet(receiver);
		}

		// Compute self position & speed
		prev_loc[robot_id][0] = loc[robot_id][0];
		prev_loc[robot_id][1] = loc[robot_id][1];

        odometry_update(time_step);
        controller_print_log();

		//update_self_motion(msl,msr);

		speed[robot_id][0] = (1/DELTA_T)*(loc[robot_id][0]-prev_loc[robot_id][0]);
		speed[robot_id][1] = (1/DELTA_T)*(loc[robot_id][1]-prev_loc[robot_id][1]);

		// Reynold's rules with all previous info (updates the speed[][] table)
		reynolds_rules();
		//printf("%f %f\n", speed[robot_id][0], speed[robot_id][1]);

		// Compute wheels speed from Reynold's speed
		compute_wheel_speeds(&msl, &msr);

		// Add Braitenberg
        braitenberg(&msl, & msr);

		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
        if (robot_id==4) printf("Just before sending: %g %g\n", msl_w, msr_w);
		wb_motor_set_velocity(dev_left_motor, msl_w);
        wb_motor_set_velocity(dev_right_motor, msr_w);

		// Send current position to neighbors, uncomment for I15, don't forget to add the declaration of "outbuffer" at the begining of this function.
		/*Implement your code here*/
		sprintf(outbuffer,"%1d#%f#%f#%f",robot_id,loc[robot_id][0],loc[robot_id][1], loc[robot_id][2]);
        wb_emitter_send(emitter,outbuffer,strlen(outbuffer));


		// Continue one step
		wb_robot_step(time_step);
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
      //time_end_calibration = wb_robot_get_time();
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
    if (VERBOSE_POS)  printf("ROBOT pose: %g %g %g\n", loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);
    //printf("ACC1: %g %g %g\n", rf[robot_id].acc.x , rf[robot_id].acc.y, rf[robot_id].acc.heading);
    Kalman_Filter(&loc[robot_id][0] , &loc[robot_id][1], &speed[robot_id][0], &speed[robot_id][1], &_meas.gps[0], &_meas.gps[1]);
    //print_cov_matrix();
    //printf("ACC2: %g %g %g\n", rf[robot_id].acc.x , rf[robot_id].acc.y, rf[robot_id].acc.heading);
    if (VERBOSE_POS)  printf("ROBOT pose after Kalman: %g %g %g\n\n", loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);
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
    //if(robot_id==4) printf("Test %g %d\n", deltaleft, isnan(deltaleft));
    if (isnan(deltaleft)==-1) deltaleft=0;
    // Store previous value of the right encoder
    _meas.prev_right_enc = _meas.right_enc;
    _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
    double deltaright=_meas.right_enc-_meas.prev_right_enc;
    if (isnan(deltaright)==-1) deltaright=0;

    //if(robot_id==4) printf("Test: %g %g\n", deltaright, deltaleft);
    deltaleft  *= WHEEL_RADIUS;
    deltaright *= WHEEL_RADIUS;

    double omega = ( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step ); //ADDED MINUS TO TEST --JEREMY
    double vel = ( deltaright + deltaleft ) / ( 2.0 * time_step );

    double a = loc[robot_id][2];
    //if(robot_id==4) printf("Angle: %g, dr: %g, dl: %g, Time step %d\n",a, deltaright, deltaleft, time_step);

    double speed_wx = vel * cos(a);
    double speed_wy = vel * sin(a);

    //A enlever à terme
    speed[robot_id][0] = speed_wx;
    speed[robot_id][1] = speed_wy;
    loc[robot_id][0] += speed[robot_id][0] * time_step;
    loc[robot_id][1] += speed[robot_id][1] * time_step;
    loc[robot_id][2] += omega * time_step;
    if (loc[robot_id][2]>M_PI) loc[robot_id][2]-=2*M_PI;
    if (loc[robot_id][2]<-M_PI) loc[robot_id][2]+=2*M_PI;
    //if(robot_id==4)
        //printf("In odometry: %g %g\n", _meas.left_enc, _meas.right_enc);

    if(VERBOSE_ENC)
        printf("ROBOT enc : x: %g  y: %g heading: %g\n", loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);
}

void controller_get_acc()
{
  int time_step = wb_robot_get_basic_time_step();

  const double * acc_values = wb_accelerometer_get_values(dev_acc);
  memcpy(_meas.acc, acc_values, sizeof(_meas.acc));

  double accfront = ( _meas.acc[1] - _meas.acc_mean[1]);
  //double accside = ( _meas.acc[0] - _meas.acc_mean[0]);
  double heading_tmp = loc[robot_id][2];

  _meas.prev_left_enc = _meas.left_enc;
  _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
  double deltaleft=_meas.left_enc-_meas.prev_left_enc;
  _meas.prev_right_enc = _meas.right_enc;
  _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);
  double deltaright=_meas.right_enc-_meas.prev_right_enc;
  deltaleft  *= WHEEL_RADIUS;
  deltaright *= WHEEL_RADIUS;
  double omega = - ( deltaright - deltaleft ) / ( WHEEL_AXIS * time_step );
  loc[robot_id][2] += omega * time_step;
  if (loc[robot_id][2]>M_PI) loc[robot_id][2]-=2*M_PI;
  if (loc[robot_id][2]<-M_PI) loc[robot_id][2]+=2*M_PI;

  double delta_heading = loc[robot_id][2] - heading_tmp;

  acc[robot_id][0] = accfront * cos(loc[robot_id][2]);
  acc[robot_id][1] = accfront * sin(loc[robot_id][2]);

  double spxtmp = speed[robot_id][0];
  double spytmp = speed[robot_id][1];
  speed[robot_id][0] = cos(delta_heading)*spxtmp - sin(delta_heading)*spytmp + acc[robot_id][0] * time_step/ 1000.0;
  speed[robot_id][1] = sin(delta_heading)*spxtmp + cos(delta_heading)*spytmp + acc[robot_id][1] * time_step/ 1000.0;
  loc[robot_id][0] += speed[robot_id][0] * time_step/ 1000.0;
  loc[robot_id][1] += speed[robot_id][1] * time_step/ 1000.0;

  if(VERBOSE_ACC)
    printf("ROBOT acc : x: %g  y: %g heading: %g\n", loc[robot_id][0], loc[robot_id][1], loc[robot_id][2]);


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
            wb_robot_get_time(), loc[robot_id][0], loc[robot_id][1] , loc[robot_id][2], _meas.gps[0], _meas.gps[2],
             speed[robot_id][0], speed[robot_id][1], acc[robot_id][0], acc[robot_id][1]);
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
