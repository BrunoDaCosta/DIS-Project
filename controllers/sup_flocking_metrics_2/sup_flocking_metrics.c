#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define MAX_ROB 5
#define TARGET_FLOCKING_DIST 0.2
#define ROBOT_MAX_SPEED 0.002

enum {POS_X=0,POS_Y,POS_Z};

static WbNodeRef robs[MAX_ROB];

const double *loc[MAX_ROB];
const double *rot[MAX_ROB];
static double center_pos_start[3], center_pos_end[3];

static void reset();
static void fitness(double* fitness, double* orientation, double* distance, double* velocity);
static void display_data(double fit, double ori, double dist, double vel);

static FILE *fp;

/* MAIN - Distribute and test conctrollers */
int main() {
    double fit=0, orientation=0, distance=0, velocity=0;
    int iter=0;
    /* Initialisation */

    fp = fopen("flocking_metrics_2.csv","w");
    fprintf(fp, "time; fitness; orientation; distance; velocity;\n");

    wb_robot_init();
    reset();

    int time_step = wb_robot_get_basic_time_step();
    while(wb_robot_step(time_step) != -1){
        fitness(&fit, &orientation, &distance, &velocity);
        //display_data(fit/((double) iter), orientation/((double) iter), distance/((double) iter), velocity/((double) iter));
        display_data(fit, orientation, distance, velocity);
        iter++;
    }
    fit/=iter; orientation/=iter; distance/=iter; velocity/=iter;
    printf("Average Performance over %d iter: \n orientation: %.5f\n distance: %.5f\n velocity: %.5f\n fitness: %.5f\n", iter, orientation, distance, velocity, fit);
    fclose(fp);
    return 0;
}

/* RESET - Get device handles and starting locations */
void reset() {
    // Device variables
    char rob[] = "epuck5";
    int i;  //counter
    center_pos_start[POS_X]=0; center_pos_start[POS_Z]=0;
    for (i=0;i<MAX_ROB;i++) {
        robs[i] = wb_supervisor_node_get_from_def(rob);
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        center_pos_start[POS_X]+=loc[i][POS_X]; center_pos_start[POS_Z]+=loc[i][POS_Z];
        rob[5]++;
    }
    center_pos_start[POS_X]/=MAX_ROB; center_pos_start[POS_Z]/=MAX_ROB;
}

// Calculate fitness
void fitness(double* fitness, double* orientation, double* distance, double* velocity){
    int i, j;
    double ori=0, dist=0, vel=0;
    // Final center of mass position
    center_pos_start[POS_X]=center_pos_end[POS_X]; center_pos_start[POS_Z]=center_pos_end[POS_Z];
    center_pos_end[POS_X]=0; center_pos_end[POS_Z]=0;
    for (i=0; i<MAX_ROB; i++){
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        center_pos_end[POS_X]+=loc[i][POS_X]; center_pos_end[POS_Z]+=loc[i][POS_Z];
    }
    center_pos_end[POS_X]/=MAX_ROB; center_pos_end[POS_Z]/=MAX_ROB;

    // ORIENTATION
    ori = 0;
    for(i=0; i<MAX_ROB-1; i++){
        for(j=i+1; j<MAX_ROB; j++){
            //printf("rot: %g %g\n");
            ori += fabs(rot[i][3]-rot[j][3])/M_PI;
        }
    }
    ori = 1.0-ori*2.0/(MAX_ROB*(MAX_ROB-1.0));

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
    dist = 1.0/(1.0+dist_to_center/MAX_ROB)*dist_between_rob*2.0/(MAX_ROB*(MAX_ROB-1.0));

    // VELOCITY
    vel = sqrt(pow(center_pos_end[POS_X]-center_pos_start[POS_X],2)+pow(center_pos_end[POS_Z]-center_pos_start[POS_Z],2));
    vel = vel/(ROBOT_MAX_SPEED);
    if (vel>1) // Patch to prevent bad init
        return;

    *orientation= ori;
    *distance = dist;
    *velocity = vel;
    *fitness = ori*dist*vel;
}

void display_data(double fit, double ori, double dist, double vel){
    char label[20];
    sprintf(label, "Avg fitness: %.5f", fit);
    wb_supervisor_set_label(0,label,0.01,0.01,0.1,0x000000,0,"Arial");
    sprintf(label, "Avg orientation: %.5f", ori);
    wb_supervisor_set_label(1,label,0.01,0.06,0.1,0x000000,0,"Arial");
    sprintf(label, "Avg distance: %.5f", dist);
    wb_supervisor_set_label(2,label,0.01,0.11,0.1,0x000000,0,"Arial");
    sprintf(label, "Avg velocity: %.5f", vel);
    wb_supervisor_set_label(3,label,0.01,0.16,0.1,0x000000,0,"Arial");

    fprintf(fp, "%g; %g; %g; %g; %g;\n", wb_robot_get_time(), fit, ori, dist, vel);
}
