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
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
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
