#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "utils.h"

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	1		// Number of robots in flock

WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag emitter;	

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

static FILE *fp;


/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	
	char rob[] = "e-puck";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"ROBOT%d",i+1);
		
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}


/*
 * Compute performance metric.
 */

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
    for (int i=0;i<FLOCK_SIZE;i++) {
       fprintf(fp, "%g; %g; %g; %g\n", wb_robot_get_time(), loc[i][0], loc[i][1] , loc[i][2]);
            }
  }
}

/*
 * Main function.
 */
 
int main() {
           if (controller_init_log("super.csv")) return 1;
           wb_robot_init();
           int time_step = wb_robot_get_basic_time_step();
	int i;			// Index
	
	reset();
	
	while (wb_robot_step(time_step) != -1) {
		
		
		
			for (i=0;i<FLOCK_SIZE;i++) {
				// Get data
                                    	loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
				loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
				loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA				
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
