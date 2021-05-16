#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "trajectories.h"
#include "utils.h"

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	1		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag emitter;	
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;		// Single emitter

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

#define RULE1_THRESHOLD 0.2
#define fit_cluster_ref 0.03
#define fit_orient_ref 1.0

static FILE *fp;

int offset;				// Offset of robots number
float migrx, migrz;			// Migration vector
float orient_migr; 			// Migration orientation
int t;

/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	emitter = wb_robot_get_device("emitter");
	if (emitter==0) printf("missing emitter\n");
	dev_left_motor = wb_robot_get_device("left wheel motor");
           dev_right_motor = wb_robot_get_device("right wheel motor");
           wb_motor_set_position(dev_left_motor, INFINITY);
           wb_motor_set_position(dev_right_motor, INFINITY);
           wb_motor_set_velocity(dev_left_motor, 0.0);
           wb_motor_set_velocity(dev_right_motor, 0.0);
	
	char rob[] = "e-puck";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"ROBOT%d",i+1+offset);
		printf("1 \n");
		printf("Rob: %s \n", rob);
		
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}


void send_init_poses(void) {
  	char buffer[255];	// Buffer for sending data
	int i;
         
	for (i=0;i<FLOCK_SIZE;i++) {
		// Get data
		loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
		loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
		loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

		// Send it out
		sprintf(buffer,"%1d#%f#%f#%f##%f#%f",i+offset,loc[i][0],loc[i][1],loc[i][2], migrx, migrz);
		wb_emitter_send(emitter,buffer,strlen(buffer));

		// Run one step
		wb_robot_step(TIME_STEP);
	}
}

/*
 * Compute performance metric.
 */
void compute_fitness(float* fit_c, float* fit_o) {
	*fit_c = 0; *fit_o = 0;
	// Compute performance indices
	// Based on distance of the robots compared to the threshold and the deviation from the perfect angle towards
	// the migration goal
	float angle_diff;
	int i; int j;
	for (i=0;i<FLOCK_SIZE;i++) {
		for (j=i+1;j<FLOCK_SIZE;j++) {	
			// Distance measure for each pair ob robots
			*fit_c += fabs(sqrtf(powf(loc[i][0]-loc[j][0],2)+powf(loc[i][1]-loc[j][1],2))-RULE1_THRESHOLD*2);
		}

		// Angle measure for each robot
		angle_diff = fabsf(loc[i][2]-orient_migr);
		*fit_o += angle_diff > M_PI ? 2*M_PI-angle_diff : angle_diff;
	}
	*fit_c /= FLOCK_SIZE*(FLOCK_SIZE+1)/2;
	*fit_o /= FLOCK_SIZE;
}

bool controller_init_log(const char* filename){
  fp = fopen(filename,"w");
  bool err = (fp == NULL);

  if( !err ){
    fprintf(fp, "time; pose_x; pose_y; pose_heading \n");
  }
  return err;
}
void controller_print_log()
{
  // TO REMOVE !!!!
  
  if( fp != NULL){
    fprintf(fp, "%g; %g; %g; %g\n",
            wb_robot_get_time(), loc[0][0], loc[0][1] , loc[0][2]);
  }
}

/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
           if (controller_init_log("super.csv")) return 1;
           wb_robot_init();
           int time_step = wb_robot_get_basic_time_step();
	char buffer[255];	// Buffer for sending data
	int i;			// Index
           printf("Value: %d \n", argc);
           printf("args: %s \n", args[1]);
           printf("args: %s \n", args[2]);
           printf("args: %s \n", args[3]);
	if (argc == 4) { // Get parameters
		offset = atoi(args[1]);
		migrx = atof(args[2]);
		migrz = atof(args[3]);
		//migration goal point comes from the controller arguments. It is defined in the world-file, under "controllerArgs" of the supervisor.
		printf("Migratory instinct : (%f, %f)\n", migrx, migrz);
	} else {
		printf("Missing argument\n");
		return 1;
	}
	
	orient_migr = -atan2f(migrx,migrz);
	if (orient_migr<0) {
		orient_migr+=2*M_PI; // Keep value within 0, 2pi
	}

	reset();

	send_init_poses();
	
	// Compute reference fitness values
	
	float fit_cluster;			// Performance metric for aggregation
	float fit_orient;			// Performance metric for orientation
		
	while (wb_robot_step(time_step) != -1) {
		
		
		if (t % 10 == 0) {
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
                                  	loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
				
				// Sending positions to the robots, comment the following two lines if you don't want the supervisor sending it                   		
				//sprintf(buffer,"%1d#%f#%f#%f##%f#%f",i+offset,loc[i][0],loc[i][1],loc[i][2], migrx, migrz);
				//wb_emitter_send(emitter,buffer,strlen(buffer));				
			}
			//Compute and normalize fitness values
			compute_fitness(&fit_cluster, &fit_orient);
			fit_cluster = fit_cluster_ref/fit_cluster;
			fit_orient = 1-fit_orient/M_PI;
			printf("time:%d, Topology Performance: %f\n", t, fit_cluster);			
		}
                      trajectory_1(dev_left_motor, dev_right_motor, 0.);
                      controller_print_log();
		t += TIME_STEP;
	}
	  
          // Close the log file
          if(fp != NULL)
          fclose(fp);
    
          // End of the simulation
          wb_robot_cleanup();
          return 0;
          
}
