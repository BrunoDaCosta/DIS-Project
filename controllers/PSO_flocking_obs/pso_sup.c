#include <stdio.h>
#include <math.h>
#include "pso.h"
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define ROBOTS 1
#define MAX_ROB 5
#define ROB_RAD 0.035
#define ARENA_SIZE .15
#define MAX_DIST 3.0
#define SENSOR_RANGE 0.3

#define NB_SENSOR 8                     // Number of proximity sensors

/* PSO definitions */
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 0.05                       // Maximum velocity particle can attain
#define MININIT 0                   // Lower bound on initialization value
#define MAXINIT 0.2                    // Upper bound on initialization value
#define ITS 3                         // Number of iterations to run

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 3000                     // Number of fitness steps to run during evolution

#define FINALRUNS 1
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8
#define N_RUNS 10

#define PI 3.1415926535897932384626433832795
enum {POS_X=0,POS_Y,POS_Z};

#define TARGET_FLOCKING_DIST 0.3
#define ROBOT_MAX_SPEED 0.002

static WbNodeRef robs[MAX_ROB];

WbDeviceTag emitter;

const double *loc[MAX_ROB];
const double *rot[MAX_ROB];
double init_loc[MAX_ROB][3];
double init_rot[MAX_ROB][4];

void calc_fitness(double[][DATASIZE],double[],int,int);
void random_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);
double robdist(int i, int j);
double rob_orientation(int i);

/* RESET - Get device handles and starting locations */
void reset(void) {
    // Device variables
    char rob[] = "epuck0";
    char em[] = "emitter_sup";
    int i;  //counter
    for (i=0;i<MAX_ROB;i++) {
        robs[i] = wb_supervisor_node_get_from_def(rob);
        //printf("robot %s\n", rob);
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        init_loc[i][0] = loc[i][0]; init_loc[i][1] = loc[i][1]; init_loc[i][2] = loc[i][2];
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        init_rot[i][0] = rot[i][0]; init_rot[i][1] = rot[i][1]; init_rot[i][2] = rot[i][2]; init_rot[i][3] = rot[i][3];
        rob[5]++;
    }
    emitter = wb_robot_get_device(em);
    if (emitter==0) printf("missing emitter %d\n",i);
}

/* MAIN - Distribute and test conctrollers */
int main() {
    double *weights;                         // Optimized result
    int i,j,k;				     // Counter variables

    /* Initialisation */
    printf("Got here\n");
    wb_robot_init();
    printf("Particle Swarm Optimization Super Controller\n");
    reset();

    wb_robot_step(256);

    double fit, w[ROBOTS][DATASIZE], f[ROBOTS];

    // Do N_RUNS runs and send the best controller found to the robot
    for (j=0;j<N_RUNS;j++) {

        // Get result of optimization
        weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,ROBOTS);

        // Set robot weights to optimization results
        fit = 0.0;
        for (i=0;i<ROBOTS;i++) {
            for (k=0;k<DATASIZE;k++){
              w[i][k] = weights[k];
            }
        }

        // Run FINALRUN tests and calculate average
        printf("Running final runs\n");
        for (i=0;i<FINALRUNS;i+=ROBOTS) {
            calc_fitness(w,f,FIT_ITS,ROBOTS);
            for (k=0;k<ROBOTS && i+k<FINALRUNS;k++) {
                //fitvals[i+k] = f[k];
                fit += f[k];
            }
        }

        fit /= FINALRUNS;  // average over the 10 runs

        printf("Average Performance: %.3f, Best weights: %g %g %g %g %g\n",fit, weights[0], weights[1], weights[2], weights[3], weights[4]*150);
    }


    /* Wait forever */
    while (1){
        calc_fitness(w,f,FIT_ITS,ROBOTS);
    }

    return 0;
}

// Randomly position specified robot
void reposition_robots() {
    int i;
    for (i=0; i<MAX_ROB; i++){
        wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"), init_loc[i]);
        wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"), init_rot[i]);
    }
}

// Calculate fitness
void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs) {
    double buffer[255];
    int i,j, iterations;
    char label[255];

    double center_pos_start[3], center_pos_end[3];

    int time_step = wb_robot_get_basic_time_step();

    double orientation, distance, velocity;

    int neg_value=0;
    buffer[0]=0;
    for (j=1;j<DATASIZE+1;j++) {
        buffer[j] = weights[0][j-1];
        if (weights[0][j-1]<0) neg_value=1;
    }

    wb_emitter_send(emitter,(void *)buffer,(DATASIZE+1)*sizeof(double));
    reposition_robots();
    wb_supervisor_simulation_reset_physics();

    center_pos_start[POS_X]=0;
    center_pos_start[POS_Z]=0;
    center_pos_end[POS_X]=0;
    center_pos_end[POS_Z]=0;
    for (i=0; i<MAX_ROB; i++){
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        center_pos_start[POS_X]+=loc[i][POS_X];
        center_pos_start[POS_Z]+=loc[i][POS_Z];
    }
    center_pos_start[POS_X]/=MAX_ROB;
    center_pos_start[POS_Z]/=MAX_ROB;


    fit[0]=0;
    for (iterations=0; iterations<its && !neg_value; iterations++){
        wb_robot_step(time_step);
        // Final center of mass position
        center_pos_start[POS_X]=center_pos_end[POS_X];
        center_pos_start[POS_Z]=center_pos_end[POS_Z];

        center_pos_end[POS_X]=0;
        center_pos_end[POS_Z]=0;
        for (i=0; i<MAX_ROB; i++){
            loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
            rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
            center_pos_end[POS_X]+=loc[i][POS_X];
            center_pos_end[POS_Z]+=loc[i][POS_Z];
        }
        center_pos_end[POS_X]/=MAX_ROB;
        center_pos_end[POS_Z]/=MAX_ROB;

        // ORIENTATION
        orientation = 0;
        for(i=0; i<MAX_ROB-1; i++){
            for(j=i+1; j<MAX_ROB; j++){
                //printf("rot: %g %g\n");
                orientation += fabs(rot[i][3]-rot[j][3])/M_PI;
            }
        }
        orientation = 1.0-orientation*2.0/(MAX_ROB*(MAX_ROB-1.0));

        // DISTANCE
        double dist_to_center = 0;
        for(i=0; i<MAX_ROB; i++){
            dist_to_center+=sqrt(pow(loc[i][POS_X]-center_pos_end[POS_X],2)+pow(loc[i][POS_Z]-center_pos_end[POS_Z],2));
        }

        double dist_between_rob = 0;
        double d=0;
        for(i=0; i<MAX_ROB-1; i++){
            for(j=i+1; j<MAX_ROB; j++){
                d=sqrt(pow(loc[i][POS_X]-loc[j][POS_X],2)+pow(loc[i][POS_Z]-loc[j][POS_Z],2));
                dist_between_rob += fmin(d/TARGET_FLOCKING_DIST, 1.0/pow(1.0-TARGET_FLOCKING_DIST+d,2));
            }
        }
        distance = 1.0/(1.0+dist_to_center/MAX_ROB)*dist_between_rob*2.0/(MAX_ROB*(MAX_ROB-1.0));

        // VEOLCITY
        velocity = sqrt(pow(center_pos_end[POS_X]-center_pos_start[POS_X],2)+pow(center_pos_end[POS_Z]-center_pos_start[POS_Z],2));
        velocity = velocity/(ROBOT_MAX_SPEED);
        if (velocity>1) // Patch to prevent bad init
            continue;

        fit[0]+=orientation*distance*velocity;
        //printf(">> Fitness: %g orientation: %g distance: %g velocity: %g\n", fit[0]/iterations, orientation, distance, velocity);
    }
    buffer[0]=1;
    wb_emitter_send(emitter,(void *)buffer,(DATASIZE+1)*sizeof(double));

    fit[0]/=its;

    //printf("Weights sent: %g %g %g --> Fitness: %g\n", weights[0][0], weights[0][1], weights[0][2], fit[0]);
    sprintf(label,"Last fitness: %.3f\n",fit[0]);
    wb_supervisor_set_label(2,label,0.01,0.95,0.05,0xffffff,0,FONT);
}

/* Optimization fitness function , used in pso.c */
/************************************************************************************/
/* Use the NEIHBORHOOD definition at the top of this file to                        */
/* change the neighborhood type for the PSO. The possible values are:               */
/* STANDARD    : Local neighborhood with 2*NB (defined above) nearest neighbors     */
/*               NEIGHBORHOOD is set to STANDARD by default                         */
/* RAND_NB     : 2*NB random neighbors                                              */
/* NCLOSE_NB   : 2*NB closest neighbors                                             */
/* FIXEDRAD_NB : All robots within a defined radius are neighbors                   */
/************************************************************************************/
void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE]) {
    calc_fitness(weights,fit,FIT_ITS,ROBOTS);
#if NEIGHBORHOOD == RAND_NB
    nRandom(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == NCLOSE_NB
    nClosest(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == FIXEDRAD_NB
    fixedRadius(neighbors,RADIUS);
#endif
}
