#ifndef NO_COGX_DEPENDENCIES

#include "ConeSlicer.hh"
#include "SpatialGridMap.hh"

using namespace SpatialGridMap;

bool ConeSlicer::startColumn(double _x, double _y) {
	x = _x;
	y = _y;

	if (minSqDist > 0.0 && (x - origin.x) * (x - origin.x) + (y - origin.y) * (y
			- origin.y) < minSqDist) {
		return false;
	}

	upH = 0;
	dnH = 0;
	upV = 0;
	dnV = 0;
	passedClipPlane = false;
	finished = false;

	currentRayX = -1;
	currentRayY = -1;

	// Check whether some planes are already bypassed at the start of the column

	// Upper bound of current update
	currentZ = zMax;

	// First find where the upper end of the column is in the view
	// cone
	Vector3 & topPoint = endPoint; // HACK use reference to avoid allocation and rewriting
	topPoint.x = x;
	topPoint.y = y;
	topPoint.z = zMax;

	// Loop through successive (in z-direction) intersections with 
	// the outer planes of the cone, until one is found that is on
	// the cone
	z1 = -DBL_MAX;
	z2 = -DBL_MAX;
	z3 = -DBL_MAX;
	z4 = -DBL_MAX;
	bool inside1 = true;
	bool inside2 = true;
	bool inside3 = true;
	bool inside4 = true;
	bool beenInside1 = false;
	bool beenInside2 = false;
	bool beenInside3 = false;
	bool beenInside4 = false;

	const double epsilon = PLANE_Z_EPSILON;

	SlicePlane &bottomPlane = horizontalPlanes.front();
	SlicePlane &topPlane = horizontalPlanes.back();
	SlicePlane &leftPlane = verticalPlanes.front();
	SlicePlane &rightPlane = verticalPlanes.back();

	// Bottom plane
	planeZPos(z1, origin, topPlane.normal, x, y);
	// Top plane
	planeZPos(z2, origin, bottomPlane.normal, x, y);
	// Leftmost plane
	planeZPos(z3, origin, leftPlane.normal, x, y);
	// Rightmost plane
	planeZPos(z4, origin, rightPlane.normal, x, y);

	do {
		optMinus(tmp, topPoint, origin);
		inside1 = dot(tmp, bottomPlane.normal) <= epsilon;
		inside2 = dot(tmp, topPlane.normal) >= -epsilon;
		inside3 = dot(tmp, leftPlane.normal) <= epsilon;
		inside4 = dot(tmp, rightPlane.normal) >= -epsilon;

		if (inside1 && inside2 && inside3 && inside4) {
			break;
		}

		if (!inside1 && beenInside1)
			return false;
		if (!inside2 && beenInside2)
			return false;
		if (!inside3 && beenInside3)
			return false;
		if (!inside4 && beenInside4)
			return false;

		beenInside1 = inside1;
		beenInside2 = inside2;
		beenInside3 = inside3;
		beenInside4 = inside4;

		switch (getNextPlane(topPoint.z, z1, z2, z3, z4)) {
		case 1:
			topPoint.z = z1;
			break;
		case 2:
			topPoint.z = z2;
			break;
		case 3:
			topPoint.z = z3;
			break;
		case 4:
			topPoint.z = z4;
			break;
		default:
			topPoint.z = zMin;
			break;
		}
	} while (topPoint.z > zMin);

	currentZ = topPoint.z;

	findCorrectRay(topPoint);

	// Reset z values for recalculation later
	z1 = -DBL_MAX;
	z2 = -DBL_MAX;
	z3 = -DBL_MAX;
	z4 = -DBL_MAX;

	return true;
}

void ConeSlicer::findCorrectRay(Vector3 & topPoint) {
	const double epsilon = PLANE_Z_EPSILON; // Planes more vertical than this

	// The ray being the first ray is a common case, we first check for this
	SlicePlane &plane1 = horizontalPlanes[0];
	optMinus(tmp, topPoint, plane1.point);
	double proj = dot(plane1.normal, tmp);
	if (proj < -epsilon) {
		currentRayY = 0; //currentRay is at least 0
		// Do a binary search to find the ray that corresponds to topPoint
		int min = 1;
		int max = 2 * heightInRays + 1;
		while (min <= max) {
			int vertRayY = (min + max) / 2;
			SlicePlane &plane = horizontalPlanes[vertRayY];
			optMinus(tmp, topPoint, plane.point);
			double proj = dot(plane.normal, tmp);
			if (proj < -epsilon) {
				//point is on the negative side of this plane
				min = vertRayY + 1;
				currentRayY = vertRayY;
			} else {
				//point is on the positive side of or on this plane
				max = vertRayY - 1;
			}
		}
	}

	if (currentRayY >= 2 * heightInRays + 1)
		currentRayY = -1;

	if (currentRayY != -1) {
		while (upH < upwardHorizontalPlanes.size() && upwardHorizontalPlanes[upH]
				!= currentRayY + 1)
			upH++;
		while (dnH < downwardHorizontalPlanes.size()
				&& downwardHorizontalPlanes[dnH] != currentRayY)
			dnH++;
	}

	// The ray being the first ray is a common case, we first check for this
	SlicePlane &plane2 = verticalPlanes[0];
	optMinus(tmp, topPoint, plane2.point);
	proj = dot(plane2.normal, tmp);
	if (proj < -epsilon) {
		currentRayX = 0;
		// Do a binary search to find the ray that corresponds to topPoint
		int min = 1;
		int max = 2 * widthInRays + 1;
		while (min <= max) {
			int horizRayX = (min + max) / 2;
			SlicePlane &plane = verticalPlanes[horizRayX];
			optMinus(tmp, topPoint, plane.point);
			double proj = dot(plane.normal, tmp);
			if (proj < -epsilon) {
				//point is on the negative side of this plane
				min = horizRayX + 1;
				currentRayX = horizRayX;
			} else {
				//point is on the positive side of or on this plane
				max = horizRayX - 1;
			}
		}
	}
	if (currentRayX >= 2 * widthInRays + 1)
		currentRayX = -1;

	if (currentRayX != -1) {
		while (upV < upwardVerticalPlanes.size() && upwardVerticalPlanes[upV]
				!= currentRayX + 1)
			upV++;
		while (dnV < downwardVerticalPlanes.size() && downwardVerticalPlanes[dnV]
				!= currentRayX)
			dnV++;
	}
}

void ConeSlicer::getNextBoundary(bool &wasInsideClip, double &nextZ) {

	const double epsilon = PLANE_Z_EPSILON; // Planes more vertical than this
	//will be treated as perfectly vertical.

	wasInsideClip = false;

	double zMin = -DBL_MAX;

	// what planes are floor and ceeling depends on angle

	double z5 = -DBL_MAX;
	// Recalclate z1-z4 if needed, otherwise use old value
	if (z1 == -DBL_MAX && upH < upwardHorizontalPlanes.size())
		planeZPos(z1, origin, horizontalPlanes[upwardHorizontalPlanes[upH]].normal,
				x, y);
	if (z2 == -DBL_MAX && dnH < downwardHorizontalPlanes.size())
		planeZPos(z2, origin,
				horizontalPlanes[downwardHorizontalPlanes[dnH]].normal, x, y);
	if (z3 == -DBL_MAX && upV < upwardVerticalPlanes.size())
		planeZPos(z3, origin, verticalPlanes[upwardVerticalPlanes[upV]].normal, x,
				y);
	if (z4 == -DBL_MAX && dnV < downwardVerticalPlanes.size())
		planeZPos(z4, origin, verticalPlanes[downwardVerticalPlanes[dnV]].normal,
				x, y);
	// We should always need to recalculate farclip plane
	if (currentRayX > -1 && currentRayY > -1) {
		planeZPos(z5, farClipPlanes[currentRayX][currentRayY].point,
				farClipPlanes[currentRayX][currentRayY].normal, x, y);
	}
	int highestPlane = getHighestPlane(currentZ, z1, z2, z3, z4, z5,
			passedClipPlane);
	int nextRayX = currentRayX;
	int nextRayY = currentRayY;

	switch (highestPlane) {
	case 1:
		nextRayY = horizontalPlanes[upwardHorizontalPlanes[upH]].indexBelowPlane;
		upH++;
		nextZ = z1;
		z1 = -DBL_MAX;
		break;
	case 2:
		nextRayY = horizontalPlanes[downwardHorizontalPlanes[dnH]].indexBelowPlane;
		dnH++;
		nextZ = z2;
		z2 = -DBL_MAX;
		break;
	case 3:
		nextRayX = verticalPlanes[upwardVerticalPlanes[upV]].indexBelowPlane;
		upV++;
		nextZ = z3;
		z3 = -DBL_MAX;
		break;
	case 4:
		nextRayX = verticalPlanes[downwardVerticalPlanes[dnV]].indexBelowPlane;
		dnV++;
		nextZ = z4;
		z4 = -DBL_MAX;
		break;
	case 5:
		nextZ = z5;
		passedClipPlane = true;
		break;
	default:
		nextZ = zMin;
	}
	if ((currentRayX >= 0 && nextRayX < 0) || nextRayX > widthInRays * 2
			|| (currentRayY >= 0 && nextRayY < 0) || nextRayY > heightInRays * 2) {
		finished = true;
	}

	if (highestPlane == 5) {
		// Passed clipping plane of current ray. Check if we were
		// going out or in
		bool clipUpper = farClipPlanes[currentRayX][currentRayY].normal.z > 0.0;
		wasInsideClip = !clipUpper;
	} else if (currentRayX > -1 && currentRayY > -1 && currentRayX
			<= heightInRays * 2 && currentRayY <= heightInRays * 2) {
		// Passed out of ray. Check if the point we're leaving
		// the previous ray is within clip range in that ray or not
		const SlicePlane &clipPlane = farClipPlanes[currentRayX][currentRayY];
		if (clipPlane.normal.z < epsilon && clipPlane.normal.z > -epsilon) {
			// Vertical clip plane. Don't use z5; compute distance
			// along normal instead
			endPoint.x = x;
			endPoint.y = y;
			endPoint.z = nextZ;

			optMinus(tmp, endPoint, clipPlane.point);
			double dist = dot(tmp, clipPlane.normal);
			wasInsideClip = dist < 0.0;
		} else {
			// Check whether intersection between clip plane
			// and z axis lies above or below the transition point
			bool clipUpper = clipPlane.normal.z > 0.0;
			wasInsideClip = clipUpper ? nextZ < z5 : nextZ > z5;
		}
	}

	if (nextZ <= zMin) {
		finished = true;
		nextZ = zMin;
	}

	if (currentRayX != nextRayX || currentRayY != nextRayY) {
		passedClipPlane = false;
	}

	currentRayX = nextRayX;
	currentRayY = nextRayY;
	currentZ = nextZ;
}

inline int ConeSlicer::getHighestPlane(double currentZ, double z1, double z2,
		double z3, double z4, double z5, bool passedClipPlane) {
	double maxZ = -DBL_MAX;
	int bestI = 0;
	if (z1 <= currentZ && z1 > maxZ) {
		maxZ = z1;
		bestI = 1;
	}
	if (z2 <= currentZ && z2 > maxZ) {
		maxZ = z2;
		bestI = 2;
	}
	if (z3 <= currentZ && z3 > maxZ) {
		maxZ = z3;
		bestI = 3;
	}
	if (z4 <= currentZ && z4 > maxZ) {
		maxZ = z4;
		bestI = 4;
	}
	if (z5 <= currentZ && z5 > maxZ && !passedClipPlane) {
		maxZ = z5;
		bestI = 5;
	}
	return bestI;
}

inline int ConeSlicer::getNextPlane(double currentZ, double z1, double z2,
		double z3, double z4) {
	double maxZ = -DBL_MAX;
	int bestI = 0;
	if (z1 < currentZ && z1 > maxZ) {
		maxZ = z1;
		bestI = 1;
	}
	if (z2 < currentZ && z2 > maxZ) {
		maxZ = z2;
		bestI = 2;
	}
	if (z3 < currentZ && z3 > maxZ) {
		maxZ = z3;
		bestI = 3;
	}
	if (z4 < currentZ && z4 > maxZ) {
		maxZ = z4;
		bestI = 4;
	}
	return bestI;
}

inline void ConeSlicer::optMinus(Vector3 & res, const Vector3 & arg1,
		const Vector3 & arg2) {
	res.x = arg1.x - arg2.x;
	res.y = arg1.y - arg2.y;
	res.z = arg1.z - arg2.z;
}

#else // Dependencies
// TODO LEGACY CODE FOR COMPILING WITHOUT DEPENDENCIES, DO NOT EDIT!!!

#include "ConeSlicer.hh"
#include "SpatialGridMap.hh"

using namespace SpatialGridMap;

bool ConeSlicer::startColumn(double _x, double _y)
{
	x = _x;
	y = _y;

	if ((x-origin(0,0))*(x-origin(0,0))+(y-origin(1,0))*(y-origin(1,0)) < 0.25) {
		//return false;
	}

	upH = 0;
	dnH = 0;
	upV = 0;
	dnV = 0;
	passedClipPlane = false;
	finished = false;

	currentRayX = -1;
	currentRayY = -1;

	// Check whether some planes are already bypassed at the start of the column

	// Upper bound of current update
	currentZ = zMax;

	// First find where the upper end of the column is in the view
	// cone
	cv::Mat_<double> & topPoint = endPoint; // HACK use reference to avoid allocation and rewriting
	topPoint(0,0) = x;
	topPoint(1,0) = y;
	topPoint(2,0) = zMax;

	// Loop through successive (in z-direction) intersections with 
	// the outer planes of the cone, until one is found that is on
	// the cone
	z1 = -DBL_MAX;
	z2 = -DBL_MAX;
	z3 = -DBL_MAX;
	z4 = -DBL_MAX;
	bool inside1 = true;
	bool inside2 = true;
	bool inside3 = true;
	bool inside4 = true;
	bool beenInside1 = false;
	bool beenInside2 = false;
	bool beenInside3 = false;
	bool beenInside4 = false;

	const double epsilon = PLANE_Z_EPSILON;

	SlicePlane &bottomPlane = horizontalPlanes.front();
	SlicePlane &topPlane = horizontalPlanes.back();
	SlicePlane &leftPlane = verticalPlanes.front();
	SlicePlane &rightPlane = verticalPlanes.back();

	// Bottom plane
	planeZPos(z1, origin,
			topPlane.normal, x, y);
	// Top plane
	planeZPos(z2, origin,
			bottomPlane.normal, x, y);
	// Leftmost plane
	planeZPos(z3, origin,
			leftPlane.normal, x, y);
	// Rightmost plane
	planeZPos(z4, origin,
			rightPlane.normal, x, y);

	do {
		optMinus(tmp, topPoint, origin);
		inside1 = tmp.dot(bottomPlane.normal) <= epsilon;
		inside2 = tmp.dot(topPlane.normal) >= -epsilon;
		inside3 = tmp.dot(leftPlane.normal) <= epsilon;
		inside4 = tmp.dot(rightPlane.normal) >= -epsilon;

		if (inside1 && inside2 && inside3 && inside4) {
			break;
		}

		if (!inside1 && beenInside1)
		return false;
		if (!inside2 && beenInside2)
		return false;
		if (!inside3 && beenInside3)
		return false;
		if (!inside4 && beenInside4)
		return false;

		beenInside1 = inside1;
		beenInside2 = inside2;
		beenInside3 = inside3;
		beenInside4 = inside4;

		switch(getNextPlane(topPoint(2,0), z1,z2,z3,z4)) {
			case 1: topPoint(2,0) = z1; break;
			case 2: topPoint(2,0) = z2; break;
			case 3: topPoint(2,0) = z3; break;
			case 4: topPoint(2,0) = z4; break;
			default: topPoint(2,0) = zMin; break;
		}
	}
	while (topPoint(2,0) > zMin);

	currentZ = topPoint(2,0);

	findCorrectRay(topPoint);

	// Reset z values for recalculation later
	z1 = -DBL_MAX;
	z2 = -DBL_MAX;
	z3 = -DBL_MAX;
	z4 = -DBL_MAX;

	return true;
}

void ConeSlicer::findCorrectRay(cv::Mat_<double> & topPoint) {
	const double epsilon = PLANE_Z_EPSILON; // Planes more vertical than this

	// The ray being the first ray is a common case, we first check for this
	SlicePlane &plane1 = horizontalPlanes[0];
	optMinus(tmp, topPoint, plane1.point);
	double proj = plane1.normal.dot(tmp);
	if (proj < -epsilon) {
		currentRayY = 0; //currentRay is at least 0
		// Do a binary search to find the ray that corresponds to topPoint
		int min = 1;
		int max = 2*heightInRays+1;
		while(min <= max) {
			int vertRayY = (min+max)/2;
			SlicePlane &plane = horizontalPlanes[vertRayY];
			optMinus(tmp, topPoint, plane.point);
			double proj = plane.normal.dot(tmp);
			if (proj < -epsilon) {
				//point is on the negative side of this plane
				min = vertRayY+1;
				currentRayY = vertRayY;
			}
			else {
				//point is on the positive side of or on this plane
				max = vertRayY-1;
			}
		}
	}

	if (currentRayY >= 2*heightInRays+1)
	currentRayY = -1;

	if (currentRayY != -1) {
		while (upH < upwardHorizontalPlanes.size() &&
				upwardHorizontalPlanes[upH] != currentRayY+1)
		upH++;
		while (dnH < downwardHorizontalPlanes.size() &&
				downwardHorizontalPlanes[dnH] != currentRayY)
		dnH++;
	}

	// The ray being the first ray is a common case, we first check for this
	SlicePlane &plane2 = verticalPlanes[0];
	optMinus(tmp, topPoint, plane2.point);
	proj = plane2.normal.dot(tmp);
	if (proj < -epsilon) {
		currentRayX = 0;
		// Do a binary search to find the ray that corresponds to topPoint
		int min = 1;
		int max = 2*widthInRays+1;
		while(min <= max) {
			int horizRayX = (min+max)/2;
			SlicePlane &plane = verticalPlanes[horizRayX];
			optMinus(tmp, topPoint, plane.point);
			double proj = plane.normal.dot(tmp);
			if (proj < -epsilon) {
				//point is on the negative side of this plane
				min = horizRayX+1;
				currentRayX = horizRayX;
			}
			else {
				//point is on the positive side of or on this plane
				max = horizRayX-1;
			}
		}
	}
	if (currentRayX >= 2*widthInRays+1)
	currentRayX = -1;

	if (currentRayX != -1) {
		while (upV < upwardVerticalPlanes.size() &&
				upwardVerticalPlanes[upV] != currentRayX+1)
		upV++;
		while (dnV < downwardVerticalPlanes.size() &&
				downwardVerticalPlanes[dnV] != currentRayX)
		dnV++;
	}
}

void ConeSlicer::getNextBoundary(
		bool &wasInsideClip,
		double &nextZ) {

	const double epsilon = PLANE_Z_EPSILON; // Planes more vertical than this
	//will be treated as perfectly vertical.

	wasInsideClip = false;

	double zMin = -DBL_MAX;

	// what planes are floor and ceeling depends on angle

	double z5 = -DBL_MAX;
	// Recalclate z1-z4 if needed, otherwise use old value
	if (z1 == -DBL_MAX && upH < upwardHorizontalPlanes.size())
	planeZPos(z1, origin,
			horizontalPlanes[upwardHorizontalPlanes[upH]].normal, x, y);
	if (z2 == -DBL_MAX && dnH < downwardHorizontalPlanes.size())
	planeZPos(z2, origin,
			horizontalPlanes[downwardHorizontalPlanes[dnH]].normal, x, y);
	if (z3 == -DBL_MAX && upV < upwardVerticalPlanes.size())
	planeZPos(z3, origin, verticalPlanes[upwardVerticalPlanes[upV]].normal, x, y);
	if (z4 == -DBL_MAX && dnV < downwardVerticalPlanes.size())
	planeZPos(z4, origin, verticalPlanes[downwardVerticalPlanes[dnV]].normal, x, y);
	// We should always need to recalculate farclip plane
	if (currentRayX > -1 && currentRayY > -1) {
		planeZPos(z5, farClipPlanes[currentRayX][currentRayY].point,
				farClipPlanes[currentRayX][currentRayY].normal, x, y);
	}
	int highestPlane = getHighestPlane(currentZ, z1,z2,z3,z4,z5, passedClipPlane);
	int nextRayX = currentRayX;
	int nextRayY = currentRayY;

	switch (highestPlane) {
		case 1:
		nextRayY = horizontalPlanes[upwardHorizontalPlanes[upH]].indexBelowPlane;
		upH++;
		nextZ = z1;
		z1 = -DBL_MAX;
		break;
		case 2:
		nextRayY = horizontalPlanes[downwardHorizontalPlanes[dnH]].indexBelowPlane;
		dnH++;
		nextZ = z2;
		z2 = -DBL_MAX;
		break;
		case 3:
		nextRayX = verticalPlanes[upwardVerticalPlanes[upV]].indexBelowPlane;
		upV++;
		nextZ = z3;
		z3 = -DBL_MAX;
		break;
		case 4:
		nextRayX = verticalPlanes[downwardVerticalPlanes[dnV]].indexBelowPlane;
		dnV++;
		nextZ = z4;
		z4 = -DBL_MAX;
		break;
		case 5:
		nextZ = z5;
		passedClipPlane = true;
		break;
		default:
		nextZ = zMin;
	}
	if ((currentRayX >= 0 && nextRayX < 0) || nextRayX > widthInRays*2 ||
			(currentRayY >= 0 && nextRayY < 0) || nextRayY > heightInRays*2) {
		finished = true;
	}

	if (highestPlane == 5) {
		// Passed clipping plane of current ray. Check if we were
		// going out or in
		bool clipUpper =
		farClipPlanes[currentRayX][currentRayY].normal(2,0) > 0.0;
		wasInsideClip = !clipUpper;
	}
	else if (currentRayX > -1 && currentRayY > -1
			&& currentRayX <= heightInRays*2 && currentRayY <= heightInRays*2) {
		// Passed out of ray. Check if the point we're leaving
		// the previous ray is within clip range in that ray or not
		const SlicePlane &clipPlane = farClipPlanes[currentRayX][currentRayY];
		if (clipPlane.normal(2,0) < epsilon &&
				clipPlane.normal(2,0) > -epsilon) {
			// Vertical clip plane. Don't use z5; compute distance
			// along normal instead
			endPoint(0,0) = x;
			endPoint(1,0) = y;
			endPoint(2,0) = nextZ;

			optMinus(tmp, endPoint, clipPlane.point);
			double dist = (tmp).dot(clipPlane.normal);
			wasInsideClip = dist < 0.0;
		}
		else {
			// Check whether intersection between clip plane
			// and z axis lies above or below the transition point
			bool clipUpper = clipPlane.normal(2,0) > 0.0;
			wasInsideClip = clipUpper ?
			nextZ < z5 : nextZ > z5;
		}
	}

	if (nextZ <= zMin) {
		finished = true;
		nextZ = zMin;
	}

	if (currentRayX != nextRayX ||
			currentRayY != nextRayY) {
		passedClipPlane = false;
	}

	currentRayX = nextRayX;
	currentRayY = nextRayY;
	currentZ = nextZ;
}

inline int ConeSlicer::getHighestPlane(double currentZ, double z1, double z2, double z3, double z4, double z5, bool passedClipPlane) {
	double maxZ = -DBL_MAX;
	int bestI = 0;
	if (z1 <= currentZ && z1 > maxZ) {
		maxZ = z1;
		bestI = 1;
	}
	if (z2 <= currentZ && z2 > maxZ) {
		maxZ = z2;
		bestI = 2;
	}
	if (z3 <= currentZ && z3 > maxZ) {
		maxZ = z3;
		bestI = 3;
	}
	if (z4 <= currentZ && z4 > maxZ) {
		maxZ = z4;
		bestI = 4;
	}
	if (z5 <= currentZ && z5 > maxZ && !passedClipPlane) {
		maxZ = z5;
		bestI = 5;
	}
	return bestI;
}

inline int ConeSlicer::getNextPlane(double currentZ, double z1, double z2, double z3, double z4) {
	double maxZ = -DBL_MAX;
	int bestI = 0;
	if (z1 < currentZ && z1 > maxZ) {
		maxZ = z1;
		bestI = 1;
	}
	if (z2 < currentZ && z2 > maxZ) {
		maxZ = z2;
		bestI = 2;
	}
	if (z3 < currentZ && z3 > maxZ) {
		maxZ = z3;
		bestI = 3;
	}
	if (z4 < currentZ && z4 > maxZ) {
		maxZ = z4;
		bestI = 4;
	}
	return bestI;
}

inline void ConeSlicer::optMinus(cv::Mat_<double> & res, const cv::Mat_<double> & arg1, const cv::Mat_<double> & arg2) {
	res(0,0) = arg1(0,0) - arg2(0,0);
	res(1,0) = arg1(1,0) - arg2(1,0);
	res(2,0) = arg1(2,0) - arg2(2,0);
}

#endif
