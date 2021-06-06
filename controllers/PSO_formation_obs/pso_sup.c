#include <stdio.h>
#include <math.h>
#include "pso.h"
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define ROBOTS 1
#define MAX_ROB 4
#define ROB_RAD 0.035
#define ARENA_SIZE .15
#define MAX_DIST 3.0
#define SENSOR_RANGE 0.3

#define NB_SENSOR 8                     // Number of proximity sensors

/* PSO definitions */
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 0.5                       // Maximum velocity particle can attain
#define MININIT 0                   // Lower bound on initialization value
#define MAXINIT 2.0                    // Upper bound on initialization value
#define ITS 2                        // Number of iterations to run

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 2000                     // Number of fitness steps to run during evolution

#define FINALRUNS 1
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8
#define N_RUNS 10

#define PI 3.1415926535897932384626433832795
enum {POS_X=0,POS_Y,POS_Z};

#define TARGET_FLOCKING_DIST 0.3
#define ROBOT_MAX_SPEED 0.002

static WbNodeRef robs[MAX_ROB];
static WbNodeRef rob_leader;

WbDeviceTag emitter;
WbDeviceTag receiver;

const double *loc[MAX_ROB+1];
const double *rot[MAX_ROB+1];
double init_loc[MAX_ROB+1][3];
double init_rot[MAX_ROB+1][4];

void calc_fitness(double[][DATASIZE],double[],int,int);
void random_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);
double robdist(int i, int j);
double rob_orientation(int i);

/* RESET - Get device handles and starting locations */
void reset() {
    // Device variables
    char rob[] = "epuck1"; // Would be epuck0 if not for the leader
    char em[] = "emitter";
    char re[] = "receiver";
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
    rob_leader = wb_supervisor_node_get_from_def("epuck0");
    //printf("robot %s\n", rob);
    loc[MAX_ROB] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(rob_leader,"translation"));
    init_loc[MAX_ROB][0] = loc[MAX_ROB][0]; init_loc[MAX_ROB][1] = loc[MAX_ROB][1]; init_loc[MAX_ROB][2] = loc[MAX_ROB][2];
    rot[MAX_ROB] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(rob_leader,"rotation"));
    init_rot[MAX_ROB][0] = rot[MAX_ROB][0]; init_rot[MAX_ROB][1] = rot[MAX_ROB][1]; init_rot[MAX_ROB][2] = rot[MAX_ROB][2]; init_rot[MAX_ROB][3] = rot[MAX_ROB][3];

    emitter = wb_robot_get_device(em);
    receiver = wb_robot_get_device(re);
    wb_receiver_enable(receiver,wb_robot_get_basic_time_step());
}

/* MAIN - Distribute and test conctrollers */
int main() {
    double *weights;                         // Optimized result
    int i,j,k;				     // Counter variables

    /* Initialisation */
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

        printf("Average Performance: %.3f, Best weights: %g %g %g\n",fit, weights[0]*2.0, weights[1]*10.0, weights[2]*1.0);
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
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(rob_leader,"translation"), init_loc[MAX_ROB]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(rob_leader,"rotation"), init_rot[MAX_ROB]);
}

// Calculate fitness
void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs) {
    double buffer[255];
    double *inbuffer;
    int i,j, iterations;
    char label[255];
    double dist[MAX_ROB];

    double center_pos_start[3], center_pos_end[3];

    int time_step = wb_robot_get_basic_time_step();

    double velocity;

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
    int good_its=its;
    for (iterations=0; iterations<its && !neg_value; iterations++){
        //printf("%d\n", iterations);
        wb_robot_step(time_step);

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

        for(i=0; i<MAX_ROB; i++)
            dist[i]=-1;

        while (wb_receiver_get_queue_length(receiver) > 0) {
            inbuffer = (double*) wb_receiver_get_data(receiver);
            dist[((int) inbuffer[0])-1]=inbuffer[1];
            //printf("%d %g\n", ((int) inbuffer[0])-1, inbuffer[1]);
            wb_receiver_next_packet(receiver);
        }

        int flag = 0;
        double distance = 0;
        for(i=0; i<MAX_ROB; i++){
            if (dist[i]==-1){
                //printf("Error in communication\n");
                flag = 1;
                iterations--;
                distance = 0;
                break;
            }else{
                distance+=dist[i];
            }
        }
        if (flag) continue;
        distance=1.0/(1.0+distance/((float) MAX_ROB));

        // VELOCITY
        velocity = sqrt(pow(center_pos_end[POS_X]-center_pos_start[POS_X],2)+pow(center_pos_end[POS_Z]-center_pos_start[POS_Z],2));
        velocity = velocity/(ROBOT_MAX_SPEED);
        if (velocity>1) // Patch to prevent bad init
            continue;

        fit[0]+=distance*velocity;
        //printf(">> Fitness: %g orientation: %g distance: %g velocity: %g\n", fit[0]/iterations, orientation, distance, velocity);
    }
    buffer[0]=1;
    //("Reset-----------------------------------------------------\n");
    wb_emitter_send(emitter,(void *)buffer,(DATASIZE+1)*sizeof(double));
    wb_robot_step(time_step);
    fit[0]/=good_its;

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
