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
#define BIAS_SPEED           200
#define DEL_SPEED            BIAS_SPEED/2


#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED_WEB     6.27    // Maximum speed webots
#define MAX_SPEED         800     // Maximum speed

#define M_PI 3.14159265358979323846
#define SIGN(x) ((x>=0)?(1):-(1))
#define SIZE_MEMORY 20

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

WbDeviceTag emitter_sup;		// Handle for the emitter node

static int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID

//-----------------------------------------------------------------------------------//
/*VARIABLES*/
static measurement_t  _meas;
static robot_t rf[FLOCK_SIZE];
static int iter = 0;

double last_gps_time_s = 0.0f;
double time_end_calibration = 0;

int Interconn[16] = {20,30,30,5,5,-5,-9,-19,-20,-10,-5,4,4,28,28,19}; // Maze
//int Interconn[16] = {20,10,5,20,20,-4,-9,-19,-20,-10,-5,20,20,4,9,19};; // Maze
//int Interconn[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // Maze
float INITIAL_POS[FLOCK_SIZE][3] = {{-2.9, 0, 0}, {-2.9, 0.1, 0}, {-2.9, -0.1, 0}, {-2.9, 0.2, 0}, {-2.9, -0.2, 0}};
float leader_old_pos[SIZE_MEMORY][2];

float migr[2] = {2, 0};	                // Migration vector

static FILE *fp;
static robot_t leader;

char* robot_name;

float goal_range   = 0.0;
float goal_bearing = 0.0;

float leader_range = 0.0;
float leader_bearing = 0.0;
float leader_orientation = 0.0;

float leader_orientation_tmp = 0.0;

int counter_kb = 0;

static float goal_x = 0.0;
static float goal_y = 0.0;

//-----------------------------------------------------------------------------------//


static void controller_get_acc();
static void controller_get_encoder();
static void controller_get_gps();
static void controller_compute_mean_acc();
static void controller_print_log();
static bool controller_init_log(const char* filename);

static void odometry_update(int time_step);
void process_received_ping_messages(int time_step);
void process_received_ping_messages_init(int time_step);
void update_leader_measurement(float new_leader_range, float new_leader_bearing, float new_leader_orientation);

static void sup_send_info();


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

    emitter_sup =  wb_robot_get_device("emitter_sup");



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

    for(i=0; i<FLOCK_SIZE; i++) {
        rf[i].pos.x = INITIAL_POS[i][0];
        rf[i].pos.y = INITIAL_POS[i][1];
        rf[i].pos.heading = INITIAL_POS[i][2];

        rf[i].rel_prev_pos.x = INITIAL_POS[i][0];
        rf[i].rel_prev_pos.y = INITIAL_POS[i][1];
    }

   for(i=0; i<SIZE_MEMORY; i++) {
        leader_old_pos[i][0]=0;
        leader_old_pos[i][1]=0;
    }

    printf("Init: robot %d\n",robot_id_u);
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
      *msl = *msl*0.01 +  bmsl/400*MAX_SPEED_WEB/1000;
      *msr = *msr*0.01 +  bmsr/400*MAX_SPEED_WEB/1000;
      if (robot_id == 2) printf("Object detected\n");
    
    }else{
      *msl += bmsl/400*MAX_SPEED_WEB/1000;
      *msr += bmsr/400*MAX_SPEED_WEB/1000;
      if (robot_id == 2) printf("No Object detected\n");
    }
    
}

void update_leader_measurement(float new_leader_range, float new_leader_bearing, float new_leader_orientation) {
	leader_range = new_leader_range;
	leader_bearing = new_leader_bearing;
	leader_orientation = new_leader_orientation;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int nsl, int nsr, float *msl, float *msr) {
	// Define constants
	float Ku = 0.2;
	float Kw = 0.5;
	float Kb = 1;
	if(counter_kb<=SIZE_MEMORY)
	{
	     Kb=0.0;
	     counter_kb+=1;
            }
	// Compute the range and bearing to the wanted position
	float x = leader_range * cosf(leader_bearing);
	float y = leader_range * sinf(leader_bearing);
	//printf("x = %f, y= %f\n",x,y);
	//float theta = leader_orientation;
	x += goal_range * cosf(- M_PI + goal_bearing /*+ theta*/);
	y += goal_range * sinf(- M_PI + goal_bearing /*+ theta*/);

	//printf("After adding: x = %f, y= %f\n",x,y);

	float range = sqrtf(x*x + y*y); // This is the wanted position (range)
	float bearing = atan2(y, x);    // This is the wanted position (bearing)

	// Compute forward control (proportional to the projected forward distance to the leader
	float u = Ku * range * cosf(bearing);
	// Compute rotional control
	float delta = leader_orientation-rf[robot_id].pos.heading;


           //if(robot_id==4) printf("leader: %f \n", leader_orientation);

           //if(robot_id==4) printf("Counter: %d \n", counter_kb);
           //if(robot_id==4) printf("Delta angle: %f \n", delta);
	if(delta>M_PI)
                delta-=2*M_PI;
	if(delta<-M_PI)
                delta+=2*M_PI;
           //if(robot_id==4) printf("Delta angle: %f \n", delta);
	float w = Kw * range * sinf(bearing) - Kb * sinf(delta);
	// Of course, we can do a lot better by accounting for the speed of the leader (rather than just the position)
           //printf("U = %f, w = %f\n",u, w);
	// Convert to wheel speeds!
	*msl = ((u - WHEEL_AXIS*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));
	*msr = ((u + WHEEL_AXIS*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS));

	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);

           *msl = ((float) *msl)*MAX_SPEED_WEB/MAX_SPEED;
           *msr = ((float) *msr)*MAX_SPEED_WEB/MAX_SPEED;

	//printf("Speed: msl = %d, msr = %d\n",(int)((u - WHEEL_AXIS*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS)), (int)((u + WHEEL_AXIS*w/2.0) / (SPEED_UNIT_RADS * WHEEL_RADIUS)));
}



int main()
{
  float msl,msr;
  //const double *message_direction;
  //double message_rssi; // Received Signal Strength indicator

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

  //read the initial packets
  int initialized = 0;

  while(!initialized){
	// Wait until leader sent range and bearing information
	while (wb_receiver_get_queue_length(receiver) == 0) {
			wb_robot_step(time_step); // Executing the simulation for 64ms
	}
	process_received_ping_messages_init(time_step);
	initialized = 1;


	}
	msl=0; msr=0;

    while (wb_robot_step(time_step) != -1)  {


 	while (wb_receiver_get_queue_length(receiver) > 0) {
                      process_received_ping_messages(time_step);

        wb_receiver_next_packet(receiver);
    }

    odometry_update(time_step);
    controller_print_log();


    compute_wheel_speeds(0, 0, &msl, &msr);



     //if(robot_id==2) printf("Before msl: %f msr: %f\n",msl,msr);
    // Add Braitenberg
    braitenberg(&msl, &msr);
    //if(robot_id==2) printf("After msl: %f msr: %f\n",msl,msr);

    limit(&msl,MAX_SPEED_WEB);
    limit(&msr,MAX_SPEED_WEB);

    wb_motor_set_velocity(dev_left_motor, msl);
    wb_motor_set_velocity(dev_right_motor, msr);

    sup_send_info();

    //wb_robot_step(time_step);               // Executing the simulation for 64ms
  }

  // Close the log file
  if(fp != NULL)
    fclose(fp);

   // End of the simulation
  wb_robot_cleanup();
  return 0;
}

void odometry_update(int time_step){
  rf[robot_id].prev_pos.x = rf[robot_id].pos.x;
  rf[robot_id].prev_pos.y = rf[robot_id].pos.y;
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
    Kalman_Filter(&rf[robot_id].pos.x , &rf[robot_id].pos.y, &rf[robot_id].speed.x, &rf[robot_id].speed.y, &_meas.gps[0], &_meas.gps[2]);
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
 void process_received_ping_messages(int time_step) {
 	float *inbuffer;	// Buffer for the receiver node
 	int other_robot_id;
 	while (wb_receiver_get_queue_length(receiver) > 0) {
 		inbuffer = (float*) wb_receiver_get_data(receiver);
 		other_robot_id = inbuffer[0];  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
                      printf("Robot_id = %d, angle = %f\n", other_robot_id, inbuffer[1]);

        wb_receiver_next_packet(receiver);
       }
 }*/

 void process_received_ping_messages_init(int time_step) {
 	const double *message_direction;
 	double message_rssi; // Received Signal Strength indicator
 	char *inbuffer;	// Buffer for the receiver node
 	int other_robot_id;

 	while (wb_receiver_get_queue_length(receiver) > 0) {
 		inbuffer = (char*) wb_receiver_get_data(receiver);
 		message_direction = wb_receiver_get_emitter_direction(receiver);
 		message_rssi = wb_receiver_get_signal_strength(receiver);

 		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
                     	double y=message_direction[0];
                      double x=-message_direction[2];

                      goal_range = sqrt(1/message_rssi);
                     	goal_bearing = -atan2(y,x);

		//printf("Goal of robot %d: range = %.2f, bearing = %.2f\n", robot_id, goal_range, goal_bearing);
		leader_range = goal_range;
		leader_bearing = goal_bearing;
		leader_orientation = 0.0;

 		leader.rel_pos.x = leader_range*cos(leader_bearing);  // relative x pos
 		leader.rel_pos.y = leader_range*sin(leader_bearing);   // relative y pos

        goal_x = leader.rel_pos.x;
        goal_y = leader.rel_pos.y;


 		//printf("Rob %d X: %f Y: %f\n", robot_id,leader.rel_pos.x,leader.rel_pos.y);

 		//if(robot_id==4)printf("Rob %d from rob %d X: %f Y: %f\n", robot_id,other_robot_id,rf[other_robot_id].rel_pos.x,rf[other_robot_id].rel_pos.y);

 		//printf("Robot %d from %d, rel_x = %f, rel_y = %f\n", robot_id, other_robot_id, rf[other_robot_id].rel_pos.x, rf[other_robot_id].rel_pos.y);

        //leader.rel_speed.x = (1/((float) time_step))*(rf[other_robot_id].rel_pos.x-rf[other_robot_id].rel_prev_pos.x);
        //leader.rel_speed.y = (1/((float) time_step))*(rf[other_robot_id].rel_pos.y-rf[other_robot_id].rel_prev_pos.y);

        //printf("    0: %f 1: %f 2: %f %f\n", message_direction[0], message_direction[1], message_direction[2], -atan2(-message_direction[2],message_direction[0]));
        wb_receiver_next_packet(receiver);
   }
 }


void process_received_ping_messages(int time_step) {
           float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
 	const double *message_direction;
 	double message_rssi; // Received Signal Strength indicator
 	char *inbuffer;	// Buffer for the receiver node
 	int other_robot_id;

 	while (wb_receiver_get_queue_length(receiver) > 0) {
       	           //if(robot_id==4) printf(" \n");

 		inbuffer = (char*) wb_receiver_get_data(receiver);
 		message_direction = wb_receiver_get_emitter_direction(receiver);
 		message_rssi = wb_receiver_get_signal_strength(receiver);

 		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
                     	double y=message_direction[0];
                      double x=-message_direction[2];


		new_leader_range = sqrt(1/message_rssi);
		new_leader_bearing = -atan2(y,x);
		double angle_pos_leader = atan2(y,x) + rf[robot_id].pos.heading;

                      leader.rel_prev_pos.x = leader.rel_pos.x;
                      leader.rel_prev_pos.y = leader.rel_pos.y;

		leader.rel_pos.x = new_leader_range*cos(new_leader_bearing);  // relative x pos
 		leader.rel_pos.y = new_leader_range*sin(new_leader_bearing);   // relative y pos

		///////////////////////////
		double leader_new_x = rf[robot_id].pos.x + new_leader_range*cos(angle_pos_leader);
		double leader_new_y = rf[robot_id].pos.y + new_leader_range*sin(angle_pos_leader);
		//if(robot_id==4) printf("NEW posrob: %f, posleadrel: %f\n",rf[robot_id].pos.y,leader.rel_pos.y);

		//////////////////
		int i = 0;
		for(i=0; i<SIZE_MEMORY-1; i++) {
                           leader_old_pos[i][0]=leader_old_pos[i+1][0];
                           leader_old_pos[i][1]=leader_old_pos[i+1][1];
                      }
		leader_old_pos[SIZE_MEMORY-1][0]=leader_new_x;
		leader_old_pos[SIZE_MEMORY-1][1]=leader_new_y;

		//////////////////////
		double leader_delta_x = leader_new_x -leader_old_pos[0][0];
		double leader_delta_y = leader_new_y -leader_old_pos[0][1];
		//if(robot_id==4) printf("ldx: %f, ldy: %f \n",leader_delta_x,leader_delta_y);
		//if(robot_id==4) printf("Delta distance: %f \n",leader_delta_x*leader_delta_x+leader_delta_y*leader_delta_y);
		if(leader_delta_x*leader_delta_x+leader_delta_y*leader_delta_y>0.00002)
		     new_leader_orientation = atan2(leader_delta_y,leader_delta_x);
		else
        		     new_leader_orientation = leader_orientation_tmp;

        		leader_orientation_tmp = new_leader_orientation;

		//if(robot_id==4) printf("Angle: %f \n",new_leader_orientation);

		double y_ref = leader.rel_prev_pos.y - leader.rel_pos.y;
		double x_ref = leader.rel_prev_pos.x - leader.rel_pos.x;

          	           //if(robot_id==4) printf("Before Range: %f, Bearing: %f, Orientation: %f\n",new_leader_range, new_leader_bearing,new_leader_orientation);
		update_leader_measurement(new_leader_range, new_leader_bearing, new_leader_orientation);




 		//break;

        //printf("    0: %f 1: %f 2: %f %f\n", message_direction[0], message_direction[1], message_direction[2], -atan2(-message_direction[2],message_direction[0]));
        wb_receiver_next_packet(receiver);
   }
 }

 // ################################# Metrics functions #################################
 /*
  *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
  *  the message contains the robot's name
  *  the range and bearing will be measured directly out of message RSSI and direction
 */
 void sup_send_info() {
   double buffer[2];
    buffer[0] = robot_id;
    buffer[1] = sqrt(pow(goal_x-leader.rel_pos.x,2) + pow(goal_y-leader.rel_pos.y,2));
    wb_emitter_send(emitter_sup,(void *)buffer,(2)*sizeof(double));;
 }
