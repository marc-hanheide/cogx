#ifndef _VECTOR
#define _VECTOR
#include <math.h>

class XVector3D {
public:
	double x, y, z, theta;
public:
	//default constructor
	XVector3D(double X = 0, double Y = 0, double Z = 0, double Theta = 0) {
		x = X;
		y = Y;
		z = Z;
		theta = Theta;
	}
	~XVector3D() {
	}
	;

	//calculate and return the magnitude of this vector
	double GetMagnitude() {
		return sqrtf(x * x + y * y + z * z);
	}

	//multiply this vector by a scalar
	XVector3D operator*(double num) const {
		return XVector3D(x * num, y * num, z * num);
	}

	//pass in a vector, pass in a scalar, return the product
	friend XVector3D operator*(double num, XVector3D const &vec) {
		return XVector3D(vec.x * num, vec.y * num, vec.z * num);
	}

	//add two vectors
	XVector3D operator+(const XVector3D &vec) const {
		return XVector3D(x + vec.x, y + vec.y, z + vec.z);
	}

	//subtract two vectors
	XVector3D operator-(const XVector3D &vec) const {
		return XVector3D(x - vec.x, y - vec.y, z - vec.z);
	}

	//normalize this vector
	void normalizeVector3D() {
		double magnitude = sqrtf(x * x + y * y + z * z);
		x /= magnitude;
		y /= magnitude;
		z /= magnitude;
	}

	//calculate and return dot product
	double dotVector3D(const XVector3D &vec) const {
		return x * vec.x + y * vec.y + z * vec.z;
	}

	//calculate and return cross product
	XVector3D crossVector3D(const XVector3D &vec) const {
		return XVector3D(y * vec.z - z * vec.y, z * vec.x - x * vec.z, x * vec.y
				- y * vec.x);
	}
};

#endif
