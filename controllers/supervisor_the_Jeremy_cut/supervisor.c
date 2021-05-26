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

  //emitter = wb_robot_get_device("emitter");
  //if (emitter==0) printf("missing emitter\n");

	
  char rob[] = "e-puck";
  int i;
  for (i=0;i<FLOCK_SIZE;i++) {
    sprintf(rob,"ROBOT%d",i+1+offset);
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
    printf("%1d x:%g y:%g heading: %g || goal: %g %g\n",i+offset,loc[i][0],loc[i][1],loc[i][2], migrx, migrz);

    //wb_emitter_send(emitter,buffer,strlen(buffer));

    // Run one step
    wb_robot_step(TIME_STEP);
  }
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
  if( fp != NULL){
    fprintf(fp, "%g; %g; %g; %g\n", wb_robot_get_time(), loc[0][0], loc[0][1] , loc[0][2]);
  }
}

/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
  if (controller_init_log("super.csv")) return 1;
  //wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();

  reset();
  //send_init_poses();
		
  while (wb_robot_step(time_step) != -1) {
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
