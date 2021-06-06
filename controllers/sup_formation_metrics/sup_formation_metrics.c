#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>

#define MAX_ROB 4
#define TARGET_FLOCKING_DIST 0.3
#define ROBOT_MAX_SPEED 0.002

enum {POS_X=0,POS_Y,POS_Z};

static WbNodeRef robs[MAX_ROB];
static WbDeviceTag receiver;

const double *loc[MAX_ROB];
const double *rot[MAX_ROB];
static double center_pos_start[3], center_pos_end[3];

static void reset();
static int fitness(double* fitness, double* distance, double* velocity);
static void display_data(double fit, double dist, double vel);

static FILE *fp;

/* MAIN - Distribute and test conctrollers */
int main() {
    double fit=0, distance=0, velocity=0;
    int iter=0;

    fp = fopen("formation_metrics.csv","w");
    fprintf(fp, "time; fitness; distance; velocity;\n");

    /* Initialisation */
    wb_robot_init();
    reset();

    int time_step = wb_robot_get_basic_time_step();
    while(wb_robot_step(time_step) != -1){
        if (!fitness(&fit, &distance, &velocity)) continue;
        display_data(fit/((double) iter), distance/((double) iter), velocity/((double) iter));
        iter++;
    }
    fit/=iter; distance/=iter; velocity/=iter;
    printf("Average Performance over %d iter: \n distance: %.5f\n velocity: %.5f\n fitness: %.5f\n", iter, distance, velocity, fit);
    fclose(fp);
    return 0;
}

/* RESET - Get device handles and starting locations */
void reset() {
    // Device variables
    char rob[] = "epuck0";
    char re[] = "receiver";
    int i;  //counter
    center_pos_start[POS_X]=0; center_pos_start[POS_Z]=0;
    for (i=0;i<MAX_ROB;i++) {
        printf("%s\n", rob);
        robs[i] = wb_supervisor_node_get_from_def(rob);
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        center_pos_start[POS_X]+=loc[i][POS_X]; center_pos_start[POS_Z]+=loc[i][POS_Z];
        rob[5]++;
    }
    center_pos_start[POS_X]/=MAX_ROB; center_pos_start[POS_Z]/=MAX_ROB;
    receiver = wb_robot_get_device(re);
    wb_receiver_enable(receiver,wb_robot_get_basic_time_step());
}

// Calculate fitness
int fitness(double* fitness, double* distance, double* velocity){
    double *inbuffer;
    int i;
    double dist[MAX_ROB];
    double vel=0;
    double total_distance = 0;

    center_pos_start[POS_X]=center_pos_end[POS_X]; center_pos_start[POS_Z]=center_pos_end[POS_Z];
    center_pos_end[POS_X]=0; center_pos_end[POS_Z]=0;
    for (i=0; i<MAX_ROB; i++){
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        center_pos_end[POS_X]+=loc[i][POS_X]; center_pos_end[POS_Z]+=loc[i][POS_Z];
    }
    center_pos_end[POS_X]/=MAX_ROB; center_pos_end[POS_Z]/=MAX_ROB;

    for(i=0; i<MAX_ROB; i++)
        dist[i]=-1;
    while (wb_receiver_get_queue_length(receiver) > 0) {

        inbuffer = (double*) wb_receiver_get_data(receiver);
        dist[((int) inbuffer[0])-1]=inbuffer[1];
        //printf("%d %g\n", (int) inbuffer[0], inbuffer[1]);
        wb_receiver_next_packet(receiver);
    }

    for(i=0; i<MAX_ROB; i++){
        if (dist[i]==-1){
            printf("Error in communication\n");
            return 0;
        }
        total_distance+=dist[i];
    }
    total_distance=1.0/(1.0+total_distance/((float) MAX_ROB));

    // VELOCITY
    vel = sqrt(pow(center_pos_end[POS_X]-center_pos_start[POS_X],2)+pow(center_pos_end[POS_Z]-center_pos_start[POS_Z],2));
    vel = vel/(ROBOT_MAX_SPEED);
    if (vel>1) // Patch to prevent bad init
        return 0;

    *distance += total_distance;
    *velocity += vel;
    *fitness += total_distance*vel;
    return 1;
}

void display_data(double fit, double dist, double vel){
    char label[20];
    sprintf(label, "Avg fitness: %.5f", fit);
    wb_supervisor_set_label(0,label,0.01,0.01,0.1,0x000000,0,"Arial");
    sprintf(label, "Avg distance: %.5f", dist);
    wb_supervisor_set_label(2,label,0.01,0.06,0.1,0x000000,0,"Arial");
    sprintf(label, "Avg velocity: %.5f", vel);
    wb_supervisor_set_label(3,label,0.01,0.11,0.1,0x000000,0,"Arial");

    fprintf(fp, "%g; %g; %g; %g;\n", wb_robot_get_time(), fit, dist, vel);
}
