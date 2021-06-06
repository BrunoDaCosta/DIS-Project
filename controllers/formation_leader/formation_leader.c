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
#include <webots/keyboard.h>

#include "kalman.h"
#include "utils.h"


// Odometry type
#define ODOMETRY_ACC false
#define ACTIVATE_KALMAN true


/*CONSTANTES*/
#define WHEEL_AXIS 	0.052 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	0.020	           // Radius of the wheel in meter
#define TIME_INIT_ACC 5                     // Time in second
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second

#define NB_SENSORS    8	           // Number of distance sensors
#define FLOCK_SIZE	1                     // Size of flock
#define BIAS_SPEED           200
#define DEL_SPEED            BIAS_SPEED/2


#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED_WEB     6.27    // Maximum speed webots
#define MAX_SPEED         800     // Maximum speed

#define RULE1_THRESHOLD     0.20           // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)      // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15          // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10) // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)      // Weight of consistency rule. default 1.0/10

#define MIGRATORY_URGE    1
#define MIGRATION_WEIGHT  (0.01/10)*20    // Wheight of attraction towards the common goal. default 0.01/10
#define MIGRATION_DIST    (0.01/10)

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
WbDeviceTag emitter;		// Handle for the emitter node

static int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

//-----------------------------------------------------------------------------------//
/*VARIABLES*/
static measurement_t  _meas;
static robot_t rf[FLOCK_SIZE];

double last_gps_time_s = 0.0f;
double time_end_calibration = 0;

int Interconn[16] = {20,30,30,5,5,-5,-9,-19,-20,-10,-5,4,4,28,28,19}; // Maze

float INITIAL_POS[FLOCK_SIZE][3] = {{-2.9, 0, 0}};

float migr[2] = {2, 0};	                // Migration vector

static FILE *fp;

char* robot_name;

//-----------------------------------------------------------------------------------//


static void controller_get_encoder();
static void controller_get_gps();

static void odometry_update(int time_step);
void process_received_ping_messages(int time_step);
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
    
    wb_keyboard_enable(ts);

    sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
    robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

    for(i=0; i<FLOCK_SIZE; i++) {
        rf[i].pos.x = INITIAL_POS[i][0];
        rf[i].pos.y = INITIAL_POS[i][1];
        rf[i].pos.heading = INITIAL_POS[i][2];

        rf[i].rel_prev_pos.x = INITIAL_POS[i][0];
        rf[i].rel_prev_pos.y = INITIAL_POS[i][1];
    }
    migr[1]=INITIAL_POS[robot_id][1];
}


void braitenberg(float* msl, float* msr){
    int i;				// Loop counter
    float factor = 5;
    float bmsl=0, bmsr=0;

    /* Braitenberg */
    for (i=0;i<NB_SENSORS;i++) {
        if(lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))!=1){
            bmsr += 200*(1/lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))) * Interconn[i] * factor;
            bmsl += 200*(1/lookuptable_sensor(wb_distance_sensor_get_value(ds[i]))) * Interconn[i+NB_SENSORS] * factor;
        }
    }
    
    if (abs(bmsr) > 0 || abs(bmsl) > 0){
      *msl = *msl*0.1 +  bmsl/400*MAX_SPEED_WEB/1000;
      *msr = *msr*0.1 +  bmsr/400*MAX_SPEED_WEB/1000;
    }else{
      *msl += bmsl/400*MAX_SPEED_WEB/1000;
      *msr += bmsr/400*MAX_SPEED_WEB/1000;
    }
    
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(float *msl, float *msr)
{
	// Compute wanted position from Reynold's speed and current location
	float Ku = 0.3;   // Forward control coefficient
	float Kw = 1.0;  // Rotational control coefficient
	float range = sqrtf(rf[robot_id].rey_speed.x*rf[robot_id].rey_speed.x +rf[robot_id].rey_speed.y*rf[robot_id].rey_speed.y);	  // Distance to the wanted position
	float bearing = atan2(rf[robot_id].rey_speed.y, rf[robot_id].rey_speed.x);	  // Orientation of the wanted position

	// Compute forward control
	float u = Ku*range*cosf(bearing-rf[robot_id].pos.heading);

	// Compute rotational control
	float w = Kw*range*sinf(bearing-rf[robot_id].pos.heading);
	
	// Convert to wheel speeds!
	*msl = (u + WHEEL_AXIS*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u - WHEEL_AXIS*w/2.0) * (1000.0 / WHEEL_RADIUS);

	//limit(msl,MAX_SPEED);
	//limit(msr,MAX_SPEED);

            *msl = ((float) *msl)*MAX_SPEED_WEB/MAX_SPEED;
            *msr = ((float) *msr)*MAX_SPEED_WEB/MAX_SPEED;
}



int main()
{
  float msl, msr;


  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  init_devices(time_step);

  while (wb_robot_step(time_step) != -1)  {
           odometry_update(time_step);
           msl=0; msr=0; // put 0 if you want to use the keyboard
            rf[robot_id].rey_speed.x = 0;
            rf[robot_id].rey_speed.y = 0;
            
            if(MIGRATORY_URGE == 1) {
		/* Implement migratory urge */
              
           
                if(fabs(migr[0]-rf[robot_id].pos.x)>MIGRATION_DIST)
                {
                  rf[robot_id].rey_speed.x += MIGRATION_WEIGHT*SIGN(migr[0]-rf[robot_id].pos.x);
                  }
                  if(fabs(migr[1]-rf[robot_id].pos.y)>500*MIGRATION_DIST)
                  {
                     rf[robot_id].rey_speed.y += MIGRATION_WEIGHT*SIGN(migr[1]-rf[robot_id].pos.y);
                 }
    	}


    // Compute wheels speed from Reynold's speed
    compute_wheel_speeds(&msl, &msr);

    
    braitenberg(&msl, &msr);
    
    limit(&msl, 6.27);
    limit(&msr, 6.27);
    
    
    wb_motor_set_velocity(dev_left_motor, msl);
    wb_motor_set_velocity(dev_right_motor, msr);
    
    
    
    send_ping();
  }

  // Close the log file
  if(fp != NULL)
    fclose(fp);

   // End of the simulation
  wb_robot_cleanup();
  return 0;
}

void odometry_update(int time_step){
    controller_get_encoder();

  KF_Update_Cov_Matrix((double) time_step/1000);

  double time_now_s = wb_robot_get_time();
  if (ACTIVATE_KALMAN &&   time_now_s - last_gps_time_s >= 1.0f){
    last_gps_time_s = time_now_s;
    controller_get_gps();
   Kalman_Filter(&rf[robot_id].pos.x , &rf[robot_id].pos.y, &rf[robot_id].speed.x, &rf[robot_id].speed.y, &_meas.gps[0], &_meas.gps[2]);
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

 }

/**
 * @brief     Get the gps measurements for the position of the robot. Get the heading angle. Fill the pose structure.
 */

void controller_get_gps(){
  memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));
  const double * gps_position = wb_gps_get_values(dev_gps);
  memcpy(_meas.gps, gps_position, sizeof(_meas.gps));
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
 




