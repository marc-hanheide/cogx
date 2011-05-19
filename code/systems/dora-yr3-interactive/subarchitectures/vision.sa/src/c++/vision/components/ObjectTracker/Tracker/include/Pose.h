 /**
 * @file Pose.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Defining a 3D pose with position and orientation.
 * @namespace Tracker
 */
 
#ifndef _POSE3_H_
#define _POSE3_H_

#include "Quaternion.h"
#include "mathlib.h"
#include "headers.h" // stdio.h, gl.h

namespace Tracking{

/**	@brief class Pose */
class Pose
{
public:
	vec3 t;					///< position
	Quaternion q;		///< rotation (represented by quaternions)
	
	/** @brief apply pose to OpenGL rendering stack*/
	void activate();
	/** @brief remove pose from OpenGL rendeing stack*/
	void deactivate();
	
	/** @brief print data to console */
	void print();
	
	/** @brief set pose by rotation matrix and position vector */
	void setPose(mat3 rot, vec3 pos);
	/** @brief get pose as rotation matrix and position vector */
	void getPose(mat3 &rot, vec3 &pos);
	/** @brief rotate pose by using euler angles */
	void rotate(float x, float y, float z);
	/** @brief rotate pose by using rotation vector */
	void rotateAxis(vec3 rot);
	void rotateEuler(vec3 rot);
	
	/** @brief move position by [x,y,z] */
	void translate(float x, float y, float z);
	
	/** @brief move position by t */
	void translate(vec3 t);

};

} // namespace Tracking

#endif
