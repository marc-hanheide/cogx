
//----------------------------------------------------------------------------
//  global declarations
//
//
//  oscar martinez mozos
//----------------------------------------------------------------------------

#ifndef __OSCAR__GLOBAL__
#define __OSCAR__GLOBAL__

#ifndef MAXDOUBLE 
#define MAXDOUBLE     1.79769313486231470e+308
#endif

#ifndef CARMEN_H

//---------------------------------------------------------
// TO DO: change these types from carmen
//---------------------------------------------------------	
typedef struct {
	double x;
	double y;
	double theta;
} carmen_point_t;	


typedef struct {
	int num_readings;
	float *range;
	char *tooclose;
	double x, y, theta;//position of the laser on the robot
	double odom_x, odom_y, odom_theta; //position of the center of the robot
	double tv, rv;
	double forward_safety_dist, side_safety_dist;
	double turn_axis;
	double timestamp;
	char host[10];
} carmen_robot_laser_message;


#endif // CARMEN_H

#endif

