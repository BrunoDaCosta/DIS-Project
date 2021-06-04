#ifndef UTILS_H
#define UTILS_H

typedef struct
{
  double x;
  double y;
  double heading;
} pose_t;

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(float *number, float limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

double lookuptable_sensor(int value){
  //return 1 if value is strange
  if(value>4095)
    return 1;


  else if(value>3474)
    return 0.005-(value-3474)/(4095-3474)*0.005;
  else if(value>2211)
    return 0.01-(value-2211)/(3473-2211)*(0.01-0.005);
  else if(value>676)
    return 0.02-(value-676)/(2211-676)*(0.02-0.01);
  else if(value>306)
    return 0.03-(value-306)/(676-306)*(0.03-0.02);
  else if(value>250)
    return 0.04-(value-250)/(306-250)*(0.03-0.04);

    //return 1 to avoid noise
  else if(value>34)
    return 1;
    //return 0.04-(value-34)/(306-34)*(0.04-0.03);
  else if(value>=0)
    return 1;
    //return 1-(value-0)/(34-0)*(1-0.04);
  else
    return 1;
}

// typedef struct
// {
//   float supervisor[3];
//   int id;
//   pose_t pos;
//   pose_t prev_pos;
//   pose_t speed;
//   pose_t acc;
//   pose_t rel_pos;
//   pose_t prev_rel_pos;
//   pose_t rel_speed;
//
//   int initialized;
// }robot_t;

#endif
