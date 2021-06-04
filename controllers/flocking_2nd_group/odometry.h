#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <string.h>
#include "utils.h"




/**
 * @brief      Initialize the logging of the file
 *
 * @param[in]  filename  The filename to write
 *
 * @return     return true if it fails
 */
bool controller_init_log(FILE **fp, int odometry_acc);

#endif
