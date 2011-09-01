 /**
 * @file tgQuaternion.h
 * @author Thomas Mörwald
 * @date September 2009
 * @version 0.1
 * @brief Quaternion representing rotations (avoiding singularity locks).
 */
 
#ifndef TG_QUATERNION
#define TG_QUATERNION

#include <math.h>

#include "tgMathlib.h"

namespace TomGine{



/** @brief Quaternion representing rotations (avoiding singularity locks). */
class tgQuaternion
{
private:
	static const float ftol = 0.0001f;
	
	
public:
	float x,y,z,w;		///< x,y,z,w	Coefficients of the quaternion.

	/** @brief Creates quaternion with coefficients (0,0,0,1). */
	tgQuaternion();
	/** @brief Creates quaternion with coefficients specified. */
	tgQuaternion(float x, float y, float z, float w);
	
	/** @brief Normalises the coefficients of the quaternion. */
	void normalise();
	/** @brief Calculates the conjungate of the quaternion. */
	tgQuaternion getConjugate() const;

	/** @brief Compares two quaternions for equality wrt. rotation. */
	bool operator==(const tgQuaternion &q) const;
	/** @brief Add coefficients. */
	tgQuaternion operator+ (const tgQuaternion &q2) const;
	/** @brief Subtract coefficients. */
	tgQuaternion operator- (const tgQuaternion &q2) const;

	/** @brief Multiplying rq with q applies the rotation q to rq. */
	tgQuaternion operator* (const tgQuaternion &rq);
	/** @brief Multiply coefficients with scalar f. */
	tgQuaternion operator* (const float f);
	
	/** @brief Multiplying quaternion q with a vector v applies the rotation to v. */
	vec3 operator* (vec3 v);
	
	/** @brief Get coefficients of quaternion from axis-angle representation. */
	void fromAxis(const vec3 &v, float angle);
	/** @brief Get coefficients of quaternion from Euler angles. */
	void fromEuler(float pitch, float yaw, float roll);
	/** @brief Get coefficients of quaternion from 3x3 matrix. */
	void fromMatrix(mat3 m);
	/** @brief Get coefficients of quaternion from 4x4 matrix. */
	void fromMatrix(mat4 m);
	/** @brief Get 4x4 matrix from quaternion representation. */
	mat4 getMatrix4() const;
	/** @brief Get 3x3 matrix from quaternion representation. */
	mat3 getMatrix3() const;
	/** @brief Get axis-angle representation from quaternion. */
	void getAxisAngle(vec3& axis, float& angle) const;
	
	/** @brief Print the coefficients of the quaternion to console. */
	void print() const;

	void printAxisAngle() const;

};

} // namespace TomGine

#endif
