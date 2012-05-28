#ifndef NO_COGX_DEPENDENCIES //Temporary for compiling code without CoGX
#ifndef CONESLICER_H
#define CONESLICER_H

#include <utility>
#include <vector>
#include <deque>
#include <cv.h>
#include <cast/core/CASTUtils.hpp>
#include <Math.hpp>
#include "Matrix33.h"
#include <algorithm> //Reverse
#define PLANE_Z_EPSILON (1e-10)
using namespace std;
using namespace cogx::Math;

namespace SpatialGridMap {

struct SlicePlane {
	Vector3 normal;
	Vector3 point;
	int indexBelowPlane;
};

template<class MapData>
class GridMap;

class ConeSlicer {
public:
	template<class MapData, class ObstacleFunctor>
	ConeSlicer(double xPos, double yPos, double zPos, double panAngle,
			double tiltAngle, double horizSizeAngle, double vertSizeAngle,
			double maxRange, int widthInRays, int heightInRays,
			const ObstacleFunctor &obstacle,
			const SpatialGridMap::GridMap<MapData> &owner, double minDistance);
	~ConeSlicer() {
	}
	;

	// finds z coordinate of a plane, if it exists
	inline bool planeZPos(double & res, const Vector3 & point,
			const Vector3 & norm, double x, double y) const;

	pair<double, double> getBoundsLower() const {
		pair<double, double> ret(xMin, yMin);
		return ret;
	}
	pair<double, double> getBoundsUpper() const {
		pair<double, double> ret(xMax, yMax);
		return ret;
	}
	bool isFinished() {
		return finished;
	}

	bool startColumn(double x, double y);
	void getNextBoundary(bool &wasInsideClip, double &nextZ);
	double getCurrentZ() {
		return currentZ;
	}
	inline int getHighestPlane(double currentZ, double z1, double z2, double z3,
			double z4, double z5, bool passedClipPlane);
	inline int getNextPlane(double currentZ, double z1, double z2, double z3,
			double z4);

private:
	void findCorrectRay(Vector3 & topPoint); // Helper for startColumn
	inline void optMinus(Vector3 & res, const Vector3 & arg1,
			const Vector3 & arg2);

	double z1;
	double z2;
	double z3;
	double z4;

	double xPos, yPos, zPos;
	double panAngle, tiltAngle;
	double horizSizeAngle, vertSizeAngle;
	double maxRange;
	double minSqDist;
	double xMin, xMax, yMin, yMax, zMax, zMin;
	int widthInRays, heightInRays;
	Vector3 origin;
	Vector3 tmp; //Prealocated for efficiency
	Vector3 endPoint; // Used in getNextBoundary, preallocated for efficiency
	vector<cv::Point2f> contour;
	vector<SlicePlane> horizontalPlanes;
	vector<SlicePlane> verticalPlanes;
	vector<int> upwardHorizontalPlanes;
	vector<int> downwardHorizontalPlanes;
	vector<int> upwardVerticalPlanes;
	vector<int> downwardVerticalPlanes;
	vector<vector<SlicePlane> > farClipPlanes;
	int upH;
	int upV;
	int dnH;
	int dnV;

	// Per-column members
	double x, y;
	bool passedClipPlane;
	double currentZ;
	bool finished;
	int currentRayX;
	int currentRayY;
};

template<class MapData, class ObstacleFunctor>
ConeSlicer::ConeSlicer(double xPos, double yPos, double zPos, double panAngle,
		double tiltAngle, double horizSizeAngle, double vertSizeAngle,
		double maxRange, int widthInRays, int heightInRays,
		const ObstacleFunctor &obstacle, const GridMap<MapData> &owner,
		double minDistance = 0.0) :
	xPos(xPos), yPos(yPos), zPos(zPos), panAngle(panAngle), tiltAngle(tiltAngle),
			horizSizeAngle(horizSizeAngle), vertSizeAngle(vertSizeAngle), maxRange(
					maxRange), widthInRays(widthInRays), heightInRays(heightInRays) {
	minSqDist = minDistance * minDistance;
	double widthFactor = 2 * tan(horizSizeAngle / 2); // Width at unit distance
	double heightFactor = 2 * tan(vertSizeAngle / 2); // Height at unit distance
	double rayWidth = widthFactor / (widthInRays * 2 + 1);
	double rayHeight = heightFactor / (heightInRays * 2 + 1);

	zMax = owner.mapZMax;
	zMin = owner.mapZMin;

	const double epsilon = PLANE_Z_EPSILON; // Planes more vertical than this
	//will be treated as perfectly vertical.

	Vector3 orth, base;
	Matrix33 rot;
	Vector3 edge[2], vertex[4];

	//Transform input to local float grid coordinates
	//Pos relative center position
	//	  xPos -= this->mapCenterX;
	//	  yPos -= this->mapCenterY;
	zPos -= owner.mapZMin;

	//Reverse rotate around origin
	//	  double xr = cos(-mapRotation) * xPos - sin(-mapRotation) * yPos;
	//	  double yr = sin(-mapRotation) * xPos + cos(-mapRotation) * yPos;

	//Cordinates relative corner
	//	  xPos = (xr + ((xGrid * cellSize) / 2)) / cellSize;
	//	  yPos = (yr + ((yGrid * cellSize) / 2)) / cellSize;

	//Transform zPos and depth to same scale for correct calculations
	//	  zPos /= cellSize;
	double maxRangeInCells = maxRange / owner.cellSize;

	//Adjust the panAngle to map rotation
	panAngle -= owner.mapRotation;

	origin.x = xPos;
	origin.y = yPos;
	origin.z = zPos;

	// we calculate generic cone first
	// disregarding position and scale
	// begin with unit vector 1,0,0
	base.x = 1;
	base.y = 0;
	base.z = 0;

	//rotate around z-axis with rotation +- width
	//this will give the angles of the cone side edges
	setZero(rot);
	rot.m22 = 1;
	for (int i = 0; i < 2; i++) {
		// r = rotation +- width / 2
		double r = panAngle - horizSizeAngle / 2.0 + horizSizeAngle * i;
		rot.m00 = cos(r);
		rot.m01 = -sin(r);
		rot.m10 = sin(r);
		rot.m11 = cos(r);
		edge[i] = rot * base;
	}

	// create a unit vector orthogonal to cone direction and z-axis (tilt angle)
	orth = base;
	setZero(rot);
	rot.m00 = cos(panAngle + M_PI / 2);
	rot.m01 = -sin(panAngle + M_PI / 2);
	rot.m10 = sin(panAngle + M_PI / 2);
	rot.m11 = cos(panAngle + M_PI / 2);
	rot.m22 = 1;
	orth = rot * base;

	{
		// rotate around this vector with tilt angle +- height
		// rotating for both edges gives position vectors for all corner vertices
		double x = orth.x, y = orth.y, z = orth.z;
		for (int i = 0; i < 2; i++) {
			double r = tiltAngle - vertSizeAngle / 2.0 + vertSizeAngle * i;

			// Build rotation matrix
			rot.m00 = 1 + (1 - cos(r)) * (x * x - 1);
			rot.m01 = (1 - cos(r)) * x * y - z * sin(r);
			rot.m02 = (1 - cos(r)) * x * z + y * sin(r);
			rot.m10 = (1 - cos(r)) * y * x + z * sin(r);
			rot.m11 = 1 + (1 - cos(r)) * (y * y - 1);
			rot.m12 = (1 - cos(r)) * y * z - x * sin(r);
			rot.m20 = (1 - cos(r)) * z * x - y * sin(r);
			rot.m21 = (1 - cos(r)) * z * y + x * sin(r);
			rot.m22 = 1 + (1 - cos(r)) * (z * z - 1);

			// Rotate both edges twice to form all four edges
			vertex[2 * i] = rot * edge[0];
			vertex[2 * i + 1] = rot * edge[1];
		}
		Matrix33 flip;

		double cp = cos(panAngle);
		double sp = sin(panAngle);
		double ct = cos(tiltAngle);
		double st = sin(tiltAngle);

		rot.m00 = cp * ct;
		rot.m01 = -sp;
		rot.m02 = -cp * st;
		rot.m10 = sp * ct;
		rot.m11 = cp;
		rot.m12 = -sp * st;
		rot.m20 = st;
		rot.m21 = 0;
		rot.m22 = ct;
		flip.m00 = 0;
		flip.m01 = 0;
		flip.m02 = -1;
		flip.m10 = -1;
		flip.m11 = 0;
		flip.m12 = 0;
		flip.m20 = 0;
		flip.m21 = 1;
		flip.m22 = 0;
		rot = rot * flip;
	}

	// calculate correct length of edge vectors 
	// extend them and set position relative origin
	double length = maxRangeInCells / cos(horizSizeAngle / 2) / cos(vertSizeAngle
			/ 2);
	for (int i = 0; i < 4; i++) {
		vertex[i] *= length;
		vertex[i] += origin;
	}

	// find the x/y contour of cone
	for (int i = 0; i < 4; i++) {
		contour.push_back(cv::Point2f(vertex[i].x, vertex[i].y));
	}
	// Push back apex
	contour.push_back(cv::Point2f(xPos, yPos));

	cv::Mat dummy;
	// Clone contour to separate memory (required for opencv)
	dummy = cv::Mat(contour).clone();
	// Call convexhull to find hull of edge points
	// (Apex should be removed if inside the corners
	convexHull(dummy, contour);

	// find x/y Min/Max in contour
	xMin = contour[0].x, xMax = xMin;
	yMin = contour[0].y, yMax = yMin;
	for (int i = 1; i < 5; i++) {
		xMin = min(xMin, (double) contour[i].x);
		xMax = max(xMax, (double) contour[i].x);
		yMin = min(yMin, (double) contour[i].y);
		yMax = max(yMax, (double) contour[i].y);
	}

	vector<vector<Vector3> > rays;
	vector<vector<double> > depths;
	//        static double a = maxRangeInCells / (widthInRays*2+1)/(heightInRays*2+1);
	for (int horizRayX = -widthInRays; horizRayX <= widthInRays; horizRayX++) {
		rays.push_back(vector<Vector3> ());
		depths.push_back(vector<double> ());

		double horizontalPoint = horizRayX * rayWidth;

		for (int vertRayY = -heightInRays; vertRayY <= heightInRays; vertRayY++) {
			double verticalPoint = vertRayY * rayHeight;
			Vector3 direction;
			direction.x = horizontalPoint;
			direction.y = verticalPoint;
			direction.z = -1;
			normalise(direction);
			direction = rot * direction;
			rays[horizRayX + widthInRays].push_back(direction);

			cv::Mat_<double> originCV(3, 1), directionCV(3, 1);
			originCV(0, 0) = origin.x;
			originCV(1, 0) = origin.y;
			originCV(2, 0) = origin.z;
			directionCV(0, 0) = direction.x;
			directionCV(1, 0) = direction.y;
			directionCV(2, 0) = direction.z;
			double rayLength = owner.rayCollisionDistance(originCV, directionCV,
					maxRange, obstacle);

			depths[horizRayX + widthInRays].push_back(rayLength);

			if (direction.x > 0 && direction.x > abs(direction.y)) {
				// Extend xMax to the radius of the range sphere
				xMax = max(xMax, origin.x + maxRangeInCells);
			}
			if (direction.x < 0 && -direction.x > abs(direction.y)) {
				// Extend xMin to the radius of the range sphere
				xMin = min(xMin, origin.x - maxRangeInCells);
			}
			if (direction.y > 0 && direction.y > abs(direction.x)) {
				// Extend yMax to the radius of the range sphere
				yMax = max(yMax, origin.y + maxRangeInCells);
			}
			if (direction.y < 0 && -direction.y > abs(direction.x)) {
				// Extend yMin to the radius of the range sphere
				yMin = min(yMin, origin.y - maxRangeInCells);
			}
		}
	}

	// Idea:
	// Create (w+1)x(h+1) slicing planes, representing the pyramids associated
	// with each cast ray.
	// Each plane either faces upwards or downwards (or neither, if 
	// vertically aligned). We can divide the planes into four groups:
	// horizontal/vertical, and upward/downward-facing.
	// Each group has a natural z-ordering, so that only four checks
	// at a time are necessary when traversing a column vertically.
	// Crossing an upwards-facing plane decrements the horizontal or
	// vertical subpyramid counter, and downwards-facing planes 
	// increment them. 
	// For each subpyramid, there is also a far clipping plane.
	// Passing that plane in the positive direction activates the update
	// and vice versa.
	// Calculate the norm vectors for all planes defining the borders of the pyramid
	for (int horizRayX = -widthInRays; horizRayX <= widthInRays + 1; horizRayX++) {
		// X coordinate of plane's intersection with the normal plane
		double horizontalPoint = horizRayX * rayWidth - 0.5 * rayWidth;

		Vector3 normal;
		normal.x = -1;
		normal.y = 0;
		normal.z = -horizontalPoint;

		normalise(normal);

		// Transform normal into cone frame
		normal = rot * normal;

		SlicePlane newPlane = { normal, origin, horizRayX + widthInRays };

		// Check whether plane points upwards, downwards or neither
		// Place plane in the appropriate vector
		if (normal.z > epsilon) {
			upwardVerticalPlanes.push_back(verticalPlanes.size());
		} else if (normal.z < -epsilon) {
			// Will be -1 for plane leading out of the
			// main cone
			newPlane.indexBelowPlane--;
			downwardVerticalPlanes.push_back(verticalPlanes.size());
		}
		verticalPlanes.push_back(newPlane);
	}
	reverse(downwardVerticalPlanes.begin(), downwardVerticalPlanes.end());

	for (int vertRayY = -heightInRays; vertRayY <= heightInRays + 1; vertRayY++) {
		// X coordinate of plane's intersection with the unit plane
		double verticalPoint = vertRayY * rayHeight - 0.5 * rayHeight;

		Vector3 normal;
		normal.x = 0;
		normal.y = -1;
		normal.z = -verticalPoint;
		normalise(normal);

		// Transform normal into cone frame
		normal = rot * normal;

		SlicePlane newPlane = { normal, origin, vertRayY + heightInRays };

		// Check whether plane points upwards, downwards or neither
		// Place plane in the appropriate vector
		if (normal.z > epsilon) {
			upwardHorizontalPlanes.push_back(horizontalPlanes.size());
		} else if (normal.z < -epsilon) {
			newPlane.indexBelowPlane--;
			// Will be -1 for plane leading out of the
			// main cone
			downwardHorizontalPlanes.push_back(horizontalPlanes.size());
		}
		horizontalPlanes.push_back(newPlane);
	}
	reverse(downwardHorizontalPlanes.begin(), downwardHorizontalPlanes.end());

	for (int horizRayX = 0; horizRayX < widthInRays * 2 + 1; horizRayX++) {
		farClipPlanes.push_back(vector<SlicePlane> ());
		for (int vertRayY = 0; vertRayY < heightInRays * 2 + 1; vertRayY++) {
			SlicePlane newPlane = { rays[horizRayX][vertRayY], origin
					+ rays[horizRayX][vertRayY] * depths[horizRayX][vertRayY], 0 };
			normalise(newPlane.normal);
			farClipPlanes[horizRayX].push_back(newPlane);
		}
	}

}

inline bool ConeSlicer::planeZPos(double & res, const Vector3 & point,
		const Vector3 & norm, double x, double y) const {
	// Plane is vertical, no z-coordinate
	if (fabs(norm.z) < 1e-8)
		return false;

	//z = (zn*z0 - nx(x-x0) + ny(y-y0)) / nz
	res = (norm.z * point.z - norm.x * (x - point.x) - norm.y * (y - point.y))
			/ norm.z;
	return true;
}

}
;

#endif //CONESLICER_H
#else // Dependencies
// TODO LEGACY CODE FOR COMPILING WITHOUT DEPENDENCIES, DO NOT EDIT!!!

#ifndef CONESLICER_H
#define CONESLICER_H

#include <utility>
#include <vector>
#include <deque>
#include <opencv/cv.h>
#include <algorithm> //Reverse
#define PLANE_Z_EPSILON (1e-10)
using namespace std;

namespace SpatialGridMap {

	struct SlicePlane
	{
		cv::Mat_<double> normal;
		cv::Mat_<double> point;
		int indexBelowPlane;
	};

	template <class MapData>
	class GridMap;

	class ConeSlicer {
	public:
		template <class MapData, class ObstacleFunctor>
		ConeSlicer(
				double xPos, double yPos, double zPos,
				double panAngle,double tiltAngle,
				double horizSizeAngle, double vertSizeAngle,
				double maxRange,
				int widthInRays, int heightInRays,
				const ObstacleFunctor &obstacle,
				const SpatialGridMap::GridMap<MapData> &owner,
				double minDistance);
		~ConeSlicer() {};

		// finds z coordinate of a plane, if it exists
		inline bool planeZPos(double & res, const cv::Mat_<double> & point, const cv::Mat_<double> & norm, double x, double y) const;

		pair<double, double> getBoundsLower() const
		{
			pair<double, double> ret(xMin, yMin);
			return ret;
		}
		pair<double, double> getBoundsUpper() const
		{
			pair<double, double> ret(xMax, yMax);
			return ret;
		}
		bool isFinished() {return finished;}

		bool startColumn(double x, double y);
		void getNextBoundary(bool &wasInsideClip, double &nextZ);
		double getCurrentZ() {return currentZ;}
		inline int getHighestPlane(double currentZ, double z1, double z2, double z3, double z4, double z5, bool passedClipPlane);
		inline int getNextPlane(double currentZ, double z1, double z2, double z3, double z4);

	private:
		void findCorrectRay(cv::Mat_<double> & topPoint); // Helper for startColumn
		inline void optMinus(cv::Mat_<double> & res, const cv::Mat_<double> & arg1, const cv::Mat_<double> & arg2);

		double z1;
		double z2;
		double z3;
		double z4;

		double xPos, yPos, zPos;
		double panAngle, tiltAngle;
		double horizSizeAngle, vertSizeAngle;
		double maxRange;
		double xMin, xMax, yMin, yMax, zMax, zMin;
		int widthInRays, heightInRays;
		cv::Mat_<double> origin;
		cv::Mat_<double> tmp; //Prealocated for efficiency
		cv::Mat_<double> endPoint; // Used in getNextBoundary, preallocated for efficiency
		vector<cv::Point2f> contour;
		vector<SlicePlane> horizontalPlanes;
		vector<SlicePlane> verticalPlanes;
		vector<int> upwardHorizontalPlanes;
		vector<int> downwardHorizontalPlanes;
		vector<int> upwardVerticalPlanes;
		vector<int> downwardVerticalPlanes;
		vector<vector<SlicePlane> > farClipPlanes;
		int upH;
		int upV;
		int dnH;
		int dnV;

		// Per-column members
		double x, y;
		bool passedClipPlane;
		double currentZ;
		bool finished;
		int currentRayX;
		int currentRayY;
	};

	template <class MapData, class ObstacleFunctor>
	ConeSlicer::ConeSlicer(
			double xPos, double yPos, double zPos,
			double panAngle,double tiltAngle,
			double horizSizeAngle, double vertSizeAngle,
			double maxRange,
			int widthInRays, int heightInRays,
			const ObstacleFunctor &obstacle,
			const SpatialGridMap::GridMap<MapData> &owner,
			double minDistance) :
	xPos(xPos), yPos(yPos), zPos(zPos),
	panAngle(panAngle), tiltAngle(tiltAngle),
	horizSizeAngle(horizSizeAngle), vertSizeAngle(vertSizeAngle),
	maxRange(maxRange), widthInRays(widthInRays), heightInRays(heightInRays),
	origin(3,1), tmp(3,1), endPoint(3,1)
	{
		double widthFactor = 2*tan(horizSizeAngle/2); // Width at unit distance
		double heightFactor = 2*tan(vertSizeAngle/2); // Height at unit distance
		double rayWidth = widthFactor / (widthInRays*2+1);
		double rayHeight = heightFactor / (heightInRays*2+1);

		zMax = owner.mapZMax;
		zMin = owner.mapZMin;

		const double epsilon = PLANE_Z_EPSILON; // Planes more vertical than this
		//will be treated as perfectly vertical.

		cv::Mat_<double> base(3,1), orth, rot(3,3);
		cv::Mat_<double> edge[2], vertex[4];

		//Transform input to local float grid coordinates
		//Pos relative center position
		//	  xPos -= this->mapCenterX;
		//	  yPos -= this->mapCenterY;
		zPos -= owner.mapZMin;

		//Reverse rotate around origin
		//	  double xr = cos(-mapRotation) * xPos - sin(-mapRotation) * yPos;
		//	  double yr = sin(-mapRotation) * xPos + cos(-mapRotation) * yPos;

		//Cordinates relative corner
		//	  xPos = (xr + ((xGrid * cellSize) / 2)) / cellSize;
		//	  yPos = (yr + ((yGrid * cellSize) / 2)) / cellSize;

		//Transform zPos and depth to same scale for correct calculations
		//	  zPos /= cellSize;
		double maxRangeInCells = maxRange / owner.cellSize;

		//Adjust the panAngle to map rotation
		panAngle -= owner.mapRotation;

		origin(0, 0) = xPos;
		origin(1, 0) = yPos;
		origin(2, 0) = zPos;

		// we calculate generic cone first
		// disregarding position and scale
		// begin with unit vector 1,0,0
		base(0, 0) = 1;
		base(1, 0) = 0;
		base(2, 0) = 0;

		//rotate around z-axis with rotation +- width
		//this will give the angles of the cone side edges
		rot.setTo(0);
		rot(2,2) = 1;
		for(int i=0; i<2; i++) {
			// r = rotation +- width / 2
			double r = panAngle - horizSizeAngle/2.0 + horizSizeAngle * i;
			rot(0,0) = cos(r);
			rot(0,1) = -sin(r);
			rot(1,0) = sin(r);
			rot(1,1) = cos(r);
			edge[i] = rot * base;
		}

		// create a unit vector orthogonal to cone direction and z-axis (tilt angle)
		orth = base.clone();
		rot.setTo(0);
		rot(0,0) = cos(panAngle+M_PI/2);
		rot(0,1) = -sin(panAngle+M_PI/2);
		rot(1,0) = sin(panAngle+M_PI/2);
		rot(1,1) = cos(panAngle+M_PI/2);
		rot(2,2) = 1;
		orth = rot * base;

		{
			// rotate around this vector with tilt angle +- height
			// rotating for both edges gives position vectors for all corner vertices
			double x = orth(0,0), y = orth(1,0), z = orth(2,0);
			for(int i=0; i<2; i++) {
				double r = tiltAngle - vertSizeAngle/2.0 + vertSizeAngle * i;

				// Build rotation matrix
				rot(0,0) = 1 + (1 - cos(r))*(x * x - 1);
				rot(0,1) = (1 - cos(r))*x*y - z * sin(r);
				rot(0,2) = (1 - cos(r))*x*z + y * sin(r);
				rot(1,0) = (1 - cos(r))*y*x + z * sin(r);
				rot(1,1) = 1 + (1 - cos(r))*(y * y - 1);
				rot(1,2) = (1 - cos(r))*y*z - x * sin(r);
				rot(2,0) = (1 - cos(r))*z*x - y * sin(r);
				rot(2,1) = (1 - cos(r))*z*y + x * sin(r);
				rot(2,2) = 1 + (1 - cos(r))*(z * z - 1);

				// Rotate both edges twice to form all four edges
				vertex[2*i] = rot * edge[0];
				vertex[2*i+1] = rot * edge[1];
			}
			cv::Mat_<double> flip(3,3);

			double cp = cos(panAngle);
			double sp = sin(panAngle);
			double ct = cos(tiltAngle);
			double st = sin(tiltAngle);

			rot(0,0) = cp*ct;
			rot(0,1) = -sp;
			rot(0,2) = -cp*st;
			rot(1,0) = sp*ct;
			rot(1,1) = cp;
			rot(1,2) = -sp*st;
			rot(2,0) = st;
			rot(2,1) = 0;
			rot(2,2) = ct;
			flip(0,0) = 0;
			flip(0,1) = 0;
			flip(0,2) = -1;
			flip(1,0) = -1;
			flip(1,1) = 0;
			flip(1,2) = 0;
			flip(2,0) = 0;
			flip(2,1) = 1;
			flip(2,2) = 0;
			rot = rot*flip;
		}

		// calculate correct length of edge vectors 
		// extend them and set position relative origin
		double length = maxRangeInCells / cos(horizSizeAngle/2) / cos(vertSizeAngle/2);
		for(int i=0; i<4; i++) {
			vertex[i] *= length;
			vertex[i] += origin;
		}

		// find the x/y contour of cone
		for(int i=0; i<4; i++) {
			contour.push_back(cv::Point2f(vertex[i](0,0), vertex[i](1,0)));
		}
		// Push back apex
		contour.push_back(cv::Point2f(xPos, yPos));

		cv::Mat dummy;
		// Clone contour to separate memory (required for opencv)
		dummy = cv::Mat(contour).clone();
		// Call convexhull to find hull of edge points
		// (Apex should be removed if inside the corners
		convexHull(dummy, contour);

		// find x/y Min/Max in contour
		xMin = contour[0].x, xMax = xMin;
		yMin = contour[0].y, yMax = yMin;
		for(int i=1; i<4; i++) {
			xMin = min(xMin, (double)contour[i].x);
			xMax = max(xMax, (double)contour[i].x);
			yMin = min(yMin, (double)contour[i].y);
			yMax = max(yMax, (double)contour[i].y);
		}

		vector<vector<cv::Mat_<double> > >rays;
		vector<vector<double> >depths;
		//        static double a = maxRangeInCells / (widthInRays*2+1)/(heightInRays*2+1);
		for (int horizRayX = -widthInRays; horizRayX <= widthInRays; horizRayX++) {
			rays.push_back(vector<cv::Mat_<double> >());
			depths.push_back(vector<double>());

			double horizontalPoint = horizRayX*rayWidth;

			for (int vertRayY = -heightInRays; vertRayY <= heightInRays; vertRayY++) {
				double verticalPoint = vertRayY*rayHeight;
				cv::Mat_<double> direction(3,1);
				direction(0, 0) = horizontalPoint;
				direction(1, 0) = verticalPoint;
				direction(2, 0) = -1;
				normalize(direction, direction);
				direction = rot * direction;
				rays[horizRayX+widthInRays].push_back(direction);

				double rayLength =
				owner.rayCollisionDistance(origin, direction, maxRange, obstacle);

				depths[horizRayX+widthInRays].push_back(rayLength);

				if (direction(0,0) > 0 && direction(0,0) > abs(direction(1,0))) {
					// Extend xMax to the radius of the range sphere
					xMax = max(xMax, origin(0,0)+maxRangeInCells);
				}
				if (direction(0,0) < 0 && -direction(0,0) > abs(direction(1,0))) {
					// Extend xMin to the radius of the range sphere
					xMin = min(xMin, origin(0,0)-maxRangeInCells);
				}
				if (direction(1,0) > 0 && direction(1,0) > abs(direction(0,0))) {
					// Extend yMax to the radius of the range sphere
					yMax = max(yMax, origin(1,0)+maxRangeInCells);
				}
				if (direction(1,0) < 0 && -direction(1,0) > abs(direction(0,0))) {
					// Extend yMin to the radius of the range sphere
					yMin = min(yMin, origin(1,0)-maxRangeInCells);
				}
			}
		}

		// Idea:
		// Create (w+1)x(h+1) slicing planes, representing the pyramids associated
		// with each cast ray.
		// Each plane either faces upwards or downwards (or neither, if 
		// vertically aligned). We can divide the planes into four groups:
		// horizontal/vertical, and upward/downward-facing.
		// Each group has a natural z-ordering, so that only four checks
		// at a time are necessary when traversing a column vertically.
		// Crossing an upwards-facing plane decrements the horizontal or
		// vertical subpyramid counter, and downwards-facing planes 
		// increment them. 
		// For each subpyramid, there is also a far clipping plane.
		// Passing that plane in the positive direction activates the update
		// and vice versa.
		// Calculate the norm vectors for all planes defining the borders of the pyramid
		for (int horizRayX = -widthInRays; horizRayX <= widthInRays+1; horizRayX++) {
			// X coordinate of plane's intersection with the normal plane
			double horizontalPoint = horizRayX*rayWidth - 0.5*rayWidth;

			cv::Mat_<double> normal(3,1);
			normal(0,0)=-1;
			normal(1,0)=0;
			normal(2,0)=-horizontalPoint;

			normalize(normal, normal);

			// Transform normal into cone frame
			normal = rot*normal;

			SlicePlane newPlane = {normal, origin, horizRayX+widthInRays};

			// Check whether plane points upwards, downwards or neither
			// Place plane in the appropriate vector
			if (normal(2,0) > epsilon) {
				upwardVerticalPlanes.push_back(verticalPlanes.size());
			}
			else if (normal(2,0) < -epsilon) {
				// Will be -1 for plane leading out of the
				// main cone
				newPlane.indexBelowPlane--;
				downwardVerticalPlanes.push_back(verticalPlanes.size());
			}
			verticalPlanes.push_back(newPlane);
		}
		reverse(downwardVerticalPlanes.begin(), downwardVerticalPlanes.end());

		for (int vertRayY = -heightInRays; vertRayY <= heightInRays+1; vertRayY++) {
			// X coordinate of plane's intersection with the unit plane
			double verticalPoint = vertRayY*rayHeight - 0.5*rayHeight;

			cv::Mat_<double> normal(3,1);
			normal(0,0) = 0;
			normal(1,0) = -1;
			normal(2,0) = -verticalPoint;
			normalize(normal,normal);

			// Transform normal into cone frame
			normal = rot*normal;

			SlicePlane newPlane = {normal, origin, vertRayY+heightInRays};

			// Check whether plane points upwards, downwards or neither
			// Place plane in the appropriate vector
			if (normal(2,0) > epsilon) {
				upwardHorizontalPlanes.push_back(horizontalPlanes.size());
			}
			else if (normal(2,0) < -epsilon) {
				newPlane.indexBelowPlane--;
				// Will be -1 for plane leading out of the
				// main cone
				downwardHorizontalPlanes.push_back(horizontalPlanes.size());
			}
			horizontalPlanes.push_back(newPlane);
		}
		reverse(downwardHorizontalPlanes.begin(), downwardHorizontalPlanes.end());

		for (int horizRayX = 0; horizRayX < widthInRays*2+1; horizRayX++) {
			farClipPlanes.push_back(vector<SlicePlane>());
			for (int vertRayY = 0; vertRayY < heightInRays*2+1; vertRayY++) {
				SlicePlane newPlane = {rays[horizRayX][vertRayY],
					origin + rays[horizRayX][vertRayY]*depths[horizRayX][vertRayY], 0};
				normalize(newPlane.normal, newPlane.normal);
				farClipPlanes[horizRayX].push_back(newPlane);
			}
		}

	}

	inline bool ConeSlicer::planeZPos(double & res, const cv::Mat_<double> & point, const cv::Mat_<double> & norm, double x, double y) const {
		// Plane is vertical, no z-coordinate
		if(fabs(norm(2,0)) < 1e-8)
		return false;

		//z = (zn*z0 - nx(x-x0) + ny(y-y0)) / nz
		res = (norm(2,0)*point(2,0) - norm(0,0)*(x-point(0,0)) - norm(1,0)*(y-point(1,0))) / norm(2,0);
		return true;
	}

};

#endif //CONESLICER_H

#endif //Temp compile
