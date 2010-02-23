#include <cast/core/CASTUtils.hpp>
#include "RelationEvaluation.hpp"
using namespace cast;
#include <Pose3.h>
#include <iostream>
  
using namespace std;
using namespace cogx::Math;

namespace spatial {

double
evaluateOnness(const Object *objectS, const Object *objectO)
{
  // Retrieve pose for supporting object (S)
  Pose3 Spose = objectS->pose;
  // Retrieve model for S
  // Retrieve pose for supported object (O)
  Pose3 Opose = objectO->pose;
  // Retrieve model for O

  int supportSurfaceType; // 0 for point, 1 for line segment, 2 for circle, 3 for polygon
  Vector3 supportSurfaceCenter; 		// For 0 and 2, epicenter
  double supportSurfaceRadius;
  std::vector<Vector3> supportSurfaceVertices; // For 1 and 3, vertices
  Vector3 supportPlaneNormal;
  double supportPlaneOffset; // dot product of plane point and plane normal

  // Find topmost surface of S:
  //	if S is a plane object, surface is a polygon or circle
  //	if S is a box, surface is the rectangle whose normal's Z-component
  //		is largest.
  //	if S is a sphere, surface is a point: the projection of the 
  //		sphere's center in O's bottom surface (defer)
  //	if S is a cylinder, surface is a circle if either end's Z-component
  //		exceeds sqrt(0.5), else it is a line segment, namely
  //		the projection of the central axis on O's bottom surface

  if (objectS->type == OBJECT_PLANE) {
    PlaneObject *plane = (PlaneObject *)objectS;
    supportPlaneNormal.z = 1.0;
    supportPlaneOffset = plane->pose.pos.z;
    if (plane->shape == PLANE_OBJECT_RECTANGLE) {
      supportSurfaceType = 3;
      double width = plane->radius1;
      double height = plane->radius2;
      Vector3 corner1 = vector3(width, height, 0.0);
      Vector3 corner2 = vector3(-width, height, 0.0);
      Vector3 corner3 = vector3(-width, -height, 0.0);
      Vector3 corner4 = vector3(width, -height, 0.0);
      supportSurfaceVertices.push_back(transform(Spose, corner1));
      supportSurfaceVertices.push_back(transform(Spose, corner2));
      supportSurfaceVertices.push_back(transform(Spose, corner3));
      supportSurfaceVertices.push_back(transform(Spose, corner4));
    }
    else { 
      supportSurfaceType = 2;
      supportSurfaceCenter = plane->pose.pos;
      supportSurfaceRadius = plane->radius1;
    }
  }
  else if (objectS->type == OBJECT_BOX) {
    BoxObject *box = (BoxObject *)objectS;

    Vector3 up = vector3(0.0, 0.0, 1.0);
    Vector3 east = vector3(1.0, 0.0, 0.0);
    Vector3 north = vector3(0.0, 1.0, 0.0);
    double z1 = transformDirection(Spose, up).z;
    double z2 = transformDirection(Spose, east).z;
    double z3 = transformDirection(Spose, north).z;
    double r1 = box->radius1;
    double r2 = box->radius2;
    double r3 = box->radius3;
    if (abs(z1) > abs(z2)) {
      if (abs(z1) > abs(z3)) {
	if (z1 > 0) {
	// Top side
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, -r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, -r2, r3)));
	}
	else {
	  // Bottom side
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, -r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, -r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, r2, -r3)));
	}
      }
      else {
	if (z3 > 0) {
	  // North side
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, r2, r3)));
	}
	else {
	  // South side
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, -r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, -r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, -r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, -r2, -r3)));
	}
      }
    }
    else {
      if (abs(z2) > abs(z3)) {
	if (z2 > 0) {
	  // East side
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, -r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, -r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, r2, -r3)));
	}
	else {
	  // West side
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, -r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, -r2, r3)));
	}
      }
      else {
	if (z3 > 0) {
	  // North side
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, r2, r3)));
	}
	else {
	  // South side
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, -r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, -r2, r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(-r1, -r2, -r3)));
	  supportSurfaceVertices.push_back(transform(Spose, vector3(r1, -r2, -r3)));
	}
      }
    }
    Vector3 side1 = supportSurfaceVertices[1] - supportSurfaceVertices[0];
    Vector3 side2 = supportSurfaceVertices[3] - supportSurfaceVertices[0];
    supportPlaneNormal = cross(side1, side2);
    normalise(supportPlaneNormal);
    supportPlaneOffset = dot(supportPlaneNormal, supportSurfaceVertices[0]);
    supportSurfaceType = 3;
  }
  else {
//    log ("Support object type not yet...supported.");
    return 0.0;
  }

  int bottomSurfaceType; // 0 for point, 1 for line segment, 2 for circle, 3 for rectangle
  Vector3 bottomSurfaceCenter; 		// For 0 and 2, epicenter
  std::vector<Vector3> bottomSurfaceVertices; // For 1 and 3, vertices
  double bottomSurfaceRadius;

  // Find bottom surface of O:
  //	if O is a box, surface is the rectangle whose normal's Z-component
  //		is most negative.
  //	if O is a sphere, surface is a point: the projection of the
  //		sphere's center in S's support surface
  //	if O is a cylinder, surface is a circle if either end's Z-component
  //		exceeds -sqrt(0.5), else it is a line segment, namely
  //		the projection of the central axis on S' supporting surface

  if (objectO->type == OBJECT_PLANE) {
    PlaneObject *plane = (PlaneObject *)objectO;
    if (plane->shape == PLANE_OBJECT_RECTANGLE) {
      bottomSurfaceType = 3;
      double width = plane->radius1;
      double height = plane->radius2;
      Vector3 corner1 = vector3(width, height, 0.0);
      Vector3 corner2 = vector3(-width, height, 0.0);
      Vector3 corner3 = vector3(-width, -height, 0.0);
      Vector3 corner4 = vector3(width, -height, 0.0);
      supportSurfaceVertices.push_back(transform(Spose, corner1));
      supportSurfaceVertices.push_back(transform(Spose, corner2));
      supportSurfaceVertices.push_back(transform(Spose, corner3));
      supportSurfaceVertices.push_back(transform(Spose, corner4));
    }
    else { 
      bottomSurfaceType = 2;
      bottomSurfaceCenter = plane->pose.pos;
      bottomSurfaceRadius = plane->radius1;
    }
  }
  else if (objectO->type == OBJECT_BOX) {
    BoxObject *box = (BoxObject *)objectO;

    Vector3 up = vector3(0.0, 0.0, 1.0);
    Vector3 east = vector3(1.0, 0.0, 0.0);
    Vector3 north = vector3(0.0, 1.0, 0.0);
    double z1 = transformDirection(Opose, up).z;
    double z2 = transformDirection(Opose, east).z;
    double z3 = transformDirection(Opose, north).z;
    double r1 = box->radius1;
    double r2 = box->radius2;
    double r3 = box->radius3;
    if (abs(z1) > abs(z2)) {
      if (abs(z1) > abs(z3)) {
	if (z1 < 0) {
	// Top side
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, -r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, -r2, r3)));
	}
	else {
	  // Bottom side
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, -r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, -r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, r2, -r3)));
	}
      }
      else {
	if (z3 < 0) {
	  // North side
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, r2, r3)));
	}
	else {
	  // South side
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, -r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, -r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, -r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, -r2, -r3)));
	}
      }
    }
    else {
      if (abs(z2) > abs(z3)) {
	if (z2 < 0) {
	  // East side
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, -r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, -r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, r2, -r3)));
	}
	else {
	  // West side
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, -r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, -r2, r3)));
	}
      }
      else {
	if (z3 < 0) {
	  // North side
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, r2, r3)));
	}
	else {
	  // South side
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, -r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, -r2, r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(-r1, -r2, -r3)));
	  bottomSurfaceVertices.push_back(transform(Opose, vector3(r1, -r2, -r3)));
	}
      }
    }
    bottomSurfaceType = 3;
  }
  else {
    //log ("Supported object type not yet...supported.");
    return 0.0;
  }

  double totalOnness = 0.0;

  // Check Center of Mass vis-a-vis projected bottom/support surfaces

  // COM projected along plane normal
  Vector3 COMNormal = supportPlaneNormal * (dot(objectO->pose.pos, supportPlaneNormal) - supportPlaneOffset);
  Vector3 COMNormalProjection = objectO->pose.pos - COMNormal;

  double COMNormalDistance = length(COMNormal);
  // COM projected along gravity direction
  Vector3 COMVertical;
  if (!equals(COMNormalDistance, 0.0, 0.001)) {
    COMVertical = (length(COMNormal) * length(COMNormal) / COMNormal.z) * vector3(0.0, 0.0, -1.0);
  }
  else {
    //COMVertical = vector3(0, 0, 0.0);
  }
  Vector3 COMVerticalProjection = COMVertical + objectO->pose.pos;

  // Compute vertical projection of bottom surface onto the support plane
  // If S is spherical or cylindrical, use the plane tangential to the
  // point/line segment.

  // If the bottom surface is a point, project the point.
  // If it is a line segment, project the endpoints.
  // If it is a polygon, project its vertices.
  // If it is a circle, project its centerpoint and
  // consider it scaled along the minor axis by the dot product
  // of the respective normals.

  // Compute contact point penalties:
  // Find the lowest point. Penalize if it is above the support plane.
  // Penalize more if it below the support plane.
  if (squareDistanceWeight != 0.0) {
    if (bottomSurfaceType == 3) {
      double minNormalDistance = FLT_MAX;
      for (std::vector<Vector3>::iterator it = bottomSurfaceVertices.begin();
	  it != bottomSurfaceVertices.end(); it++) {
	double z0 = dot(*it, supportPlaneNormal) - supportPlaneOffset;
	if (z0 < minNormalDistance) {
	  minNormalDistance = z0;
	}
      }

      double onnessFactor;
      if (minNormalDistance > 0.0) {
	onnessFactor = 1/(1+minNormalDistance*minNormalDistance/(squareDistanceFalloff*squareDistanceFalloff));
      }
      else {
	// TODO: differentiate above/below cases
	onnessFactor = 1/(1+minNormalDistance*minNormalDistance/(squareDistanceFalloff*squareDistanceFalloff));
      }

      if (onnessFactor == 0.0) {
	return 0.0;
      }
      else {
	totalOnness += log(onnessFactor) * squareDistanceWeight;
      }
    }
    else {
      //log("Bottom surface type not yet supported!");
      return 0.0;
    }
  }

  // Compute COM projection/support surface penalties
  // Compute vertical projection of COM on the support plane
  // If support surface is a polygon, find the closest edge to the COM:
  // 	Find the normal distance to the line, and the distance to the
  //	edge points. Find the position along the edge. If it is between
  //	the end points, use the normal distance, otherwise the end point
  //	distance nearest. If normal distance is used, it can be
  //	positive or negative depending on the winding of the polygon. 
  //	(Assumes convex polygons)
  // If support surface is an ellipse, find squish factor (ratio of
  //	minor to major axis); enlarge one coordinate of COM projection
  //	by this amount, then compute distance to center and subtract
  //	major axis. (Not exact, but should be good enough for factors
  //	< sqrt(2))
  // Do this analogously for the projection of the bottom surface of O.

  if (supportCOMContainmentWeight != 0.0) {
    if (supportSurfaceType == 3) {
      bool inside = false;
      double closestDistance = FLT_MAX;

      // Compare COM position to each edge of support polygon
      // Note: All geometry is flattened into the XY-plane
      for (unsigned i = 0; i < supportSurfaceVertices.size(); i++) {
	unsigned iplus = (i+1 == supportSurfaceVertices.size()) ? 0 : i+1;

	Vector3 side = supportSurfaceVertices[iplus] - supportSurfaceVertices[i];
	side.z = 0.0;
	double sideLength = length(side);
	normalise(side);
	Vector3 positionAlong = COMVerticalProjection - supportSurfaceVertices[i];
	positionAlong.z = 0.0;
	double lengthAlong = dot(positionAlong, side);

	if (lengthAlong < 0) {
	  // COM is not along the edge but past the "hither" vertex
	  double distance = length(positionAlong);
	  if (distance < closestDistance) {
	    closestDistance = distance;
	    inside = false;	// Only valid if polygon is convex!
	  }
	}
	else if (lengthAlong > sideLength) {
	  // COM is not along the edge but past the "further" vertex
	  // This case will be dealt with next iteraton
	}
	else {
	  // COM is closest to this edge by the normal distance
	  Vector3 normalDistanceVector = (positionAlong - lengthAlong*side);
	  double normalDistance = length(normalDistanceVector);
	  if (normalDistance < closestDistance) {
	    closestDistance = normalDistance;
	    // The cross product between the side vector and the normal vector
	    // points outwards
	    inside = dot(normalDistanceVector, cross(side, vector3(0.0, 0.0, 1.0))) < 0;
	  }
	}
      }

      // Onness by COM distance to support surface: Distance to nearest edge
      // divided by maximum distance possible
      double distanceFactor = closestDistance / 
	getMaxPolygonClearance(supportSurfaceVertices);
      double onnessFactor;
      if (inside) {
	onnessFactor =
	  (1 + exp(-supportCOMContainmentSteepness*(1 - supportCOMContainmentOffset)))
	  / (1 + exp(-supportCOMContainmentSteepness*
		(distanceFactor - supportCOMContainmentOffset)));
      }
      else {
	onnessFactor =
	  (1 + exp(-supportCOMContainmentSteepness*(1 - supportCOMContainmentOffset))) 
	  / (1 + exp(-supportCOMContainmentSteepness*
		(-distanceFactor - supportCOMContainmentOffset)));
      }
      totalOnness += log(onnessFactor) * supportCOMContainmentWeight;
    }
    else if (supportSurfaceType == 2) {
      // Circular support surface
      Vector3 radiusVector = COMVerticalProjection - supportSurfaceCenter;
      radiusVector.z = 0.0;


      // Can't be bothered to compute the exact distance to the ellipse. It's a 
      // quartic polynomial. I'll just approximate. Should work for small 
      // eccentricities.
      Vector3 minorAxis = supportPlaneNormal;
      minorAxis.z = 0.0;
      if (minorAxis != vector3(0.0, 0.0, 0.0)) {
	double squashFactor = normalise(minorAxis);

	double minorDistance = dot(radiusVector, minorAxis); // COM projection along minor axis
	double majorDistance = length((radiusVector - minorDistance * minorAxis));
	double approxCos = majorDistance / length(radiusVector);  
	double approxSinSq = (1 - approxCos*approxCos) * squashFactor*squashFactor;
	double approxDistance = length(radiusVector) - supportSurfaceRadius * sqrt(approxCos*approxCos + approxSinSq);


	// Onness by COM distance to support surface: Distance to nearest edge
	// divided by maximum distance possible
	double distanceFactor = 
	  approxDistance / supportSurfaceRadius;

	double onnessFactor = 
	  (1 + exp(-supportCOMContainmentSteepness*(1 - supportCOMContainmentOffset))) 
	  / (1 + exp(-supportCOMContainmentSteepness*
		(-distanceFactor - supportCOMContainmentOffset)));
	totalOnness += log(onnessFactor) * supportCOMContainmentWeight;
      }
      else {
	// Circle
	double distanceFactor = length(radiusVector) / supportSurfaceRadius - 1;
	double onnessFactor = 
	  (1 + exp(-supportCOMContainmentSteepness*(1 - supportCOMContainmentOffset))) 
	  / (1 + exp(-supportCOMContainmentSteepness*
		(-distanceFactor - supportCOMContainmentOffset)));
	totalOnness += log(onnessFactor) * supportCOMContainmentWeight;
      }
    }
    else {
      //log("Support surface type not supported yet!");
      return 0.0;
    }
  }

  if (bottomCOMContainmentWeight) {
    if (bottomSurfaceType == 3) {
      bool inside = false;
      double closestDistance = FLT_MAX;

      // Compare COM position to each edge of bottom polygon
      // Note: All geometry is flattened into the XY-plane
      for (unsigned int i = 0; i < bottomSurfaceVertices.size(); i++) {
	unsigned int iplus = (i+1 == bottomSurfaceVertices.size()) ? 0 : i+1;

	Vector3 side = bottomSurfaceVertices[iplus] - bottomSurfaceVertices[i];
	side.z = 0.0;
	double sideLength = length(side);
	normalise(side);
	Vector3 positionAlong = COMVerticalProjection - bottomSurfaceVertices[i];
	positionAlong.z = 0.0;
	double lengthAlong = dot(positionAlong, side);

	if (lengthAlong < 0) {
	  // COM is not along the edge but past the "hither" vertex
	  double distance = length(positionAlong);
	  if (distance < closestDistance) {
	    closestDistance = distance;
	    inside = false;	// Only valid if polygon is convex!
	  }
	}
	else if (lengthAlong > sideLength) {
	  // COM is not along the edge but past the "further" vertex
	  // This case will be dealt with next iteraton
	}
	else {
	  // COM is closest to this edge by the normal distance
	  Vector3 normalDistanceVector = (positionAlong - lengthAlong*side);
	  double normalDistance = length(normalDistanceVector);
	  if (normalDistance < closestDistance) {
	    closestDistance = normalDistance;
	    // The cross product between the side vector and the normal vector
	    // points outwards
	    inside = dot(normalDistanceVector, cross(side, vector3(0.0, 0.0, -1.0))) < 0;
	  }
	}
      }

      double distanceFactor = closestDistance / 
	getMaxPolygonClearance(bottomSurfaceVertices);
      double onnessFactor;
      if (inside) {
	onnessFactor =
	  (1 + exp(-bottomCOMContainmentSteepness*(1 - bottomCOMContainmentOffset)))
	  / (1 + exp(-bottomCOMContainmentSteepness*
		(distanceFactor - bottomCOMContainmentOffset)));
      }
      else {
	onnessFactor =
	  (1 + exp(-bottomCOMContainmentSteepness*(1 - bottomCOMContainmentOffset))) 
	  / (1 + exp(-bottomCOMContainmentSteepness*
		(-distanceFactor - bottomCOMContainmentOffset)));
      }
      totalOnness += log(onnessFactor) * bottomCOMContainmentWeight;
    }
    else {
      //log("Bottom surface type not supported yet!");
      return 0.0;
    }
  }

  // Impose a large penalty for COMs outside the support surface. 
  // 	(Highly unstable)
  // Impose a large penalty for COMs outside the bottom surface
  //	(Highly unstable)
  // Impose a small penalty for COMs near the inner edge of the
  // 	support or bottom surface
  //	(Close to being unstable)

  // Impose a penalty dependent on the inclination of the support plane
  if (planeInclinationWeight != 0.0) {
    double supportPlaneDisinclination = 
      supportPlaneNormal.z*supportPlaneNormal.z;

    if (supportPlaneDisinclination == 0.0) {
      return 0.0;
    }
    totalOnness += log(supportPlaneDisinclination) * planeInclinationWeight;
  }

  if (overlapWeight != 0.0) {
    //Project bottom surface onto support surface plane
    if (bottomSurfaceType == 3) {
      std::vector<Vector3> projectedBottomVertices;
      projectedBottomVertices.reserve(bottomSurfaceVertices.size());

      //Need to reverse the winding on the bottom polygon for overlap compoutation
      for (int i = bottomSurfaceVertices.size()-1; i >= 0; i--) {
	projectedBottomVertices.push_back(bottomSurfaceVertices[i]
	    -supportPlaneNormal*(dot(bottomSurfaceVertices[i],supportPlaneNormal) - supportPlaneOffset));
      }
      double maxArea =  getPolygonArea(projectedBottomVertices);

      if (supportSurfaceType == 3) {
	double overlapArea = getPolygonArea(
	    findPolygonIntersection(projectedBottomVertices, supportSurfaceVertices));
	if (overlapArea == 0.0) {
	  return 0.0;
	}
	totalOnness += log(overlapArea/maxArea) * overlapWeight;
      }

      else if (supportSurfaceType == 2) {
	double overlapArea = findOverlappingArea(projectedBottomVertices,
	    supportSurfaceCenter, supportSurfaceRadius, supportPlaneNormal);
	if (overlapArea == 0.0) {
	  return 0.0;
	}
	totalOnness += log(overlapArea/maxArea) * overlapWeight;
      }

      else {
	//log("Support surface type not supported yet");
	return 0.0;
      }
    }
    else {
      //log("Bottom surface type not supported yet");
      return 0.0;
    }
  }

  double totalWeights = squareDistanceWeight + supportCOMContainmentWeight +
    bottomCOMContainmentWeight + planeInclinationWeight + overlapWeight;

  return totalWeights == 0.0 ? 0.0 : exp(totalOnness / totalWeights);
}

std::vector<Vector3>
findPolygonIntersection(const std::vector<Vector3> &polygon1, 
    const std::vector<Vector3> &polygon2)
{
  // Find all vertices of either polygon that is strictly inside the other,
  // and all intersection points. Then find the convex hull around these
  // interest points.
  std::vector<Vector3> interestPoints;

  double epsilon = 0.0001;

  // Points of polygon 1 inside polygon 2
  for (unsigned int i = 0; i < polygon1.size(); i++) {
    bool vertex_i_inside_polygon_2 = true;
    for (unsigned int j = 0; j < polygon2.size(); j++) {
      unsigned int jplus = (j == polygon2.size()-1 ? 0 : j + 1);

      if (cross(polygon2[jplus]-polygon2[j], polygon1[i]-polygon2[j]).z < 0.0) {
	vertex_i_inside_polygon_2 = false;
	break;
      }
    }
    if (vertex_i_inside_polygon_2) {
      interestPoints.push_back(polygon1[i]);
    }
  }
  // Points of polygon 2 inside polygon 1
  for (unsigned int i = 0; i < polygon2.size(); i++) {
    bool vertex_i_inside_polygon_1 = true;
    for (unsigned int j = 0; j < polygon1.size(); j++) {
      unsigned int jplus = (j == polygon1.size()-1 ? 0 : j + 1);

      if (cross(polygon1[jplus]-polygon1[j], polygon2[i]-polygon1[j]).z < 0.0) {
	vertex_i_inside_polygon_1 = false;
	break;
      }
    }
    if (vertex_i_inside_polygon_1) {
      interestPoints.push_back(polygon2[i]);
    }
  }

  for (unsigned int i = 0; i < polygon1.size(); i++)  {
    unsigned int iplus = (i == polygon1.size()-1 ? 0 : i + 1);
    Vector3 currentPoint = polygon1[i];
    Vector3 thisEdge = polygon1[iplus] - currentPoint;

    for (unsigned int j = 0; j < polygon2.size(); j++) {
      unsigned int jplus = (j == polygon2.size()-1 ? 0 : j + 1);

      Vector3 difference;
      Vector3 otherEdgeDir;
      double otherEdgeLength;
      difference = polygon2[j] - currentPoint;
      otherEdgeDir = polygon2[jplus] - polygon2[j];

      if (abs(dot(thisEdge, otherEdgeDir)) == 1.0) {
	continue;
      }

      otherEdgeLength = length(otherEdgeDir);
      otherEdgeDir /= otherEdgeLength;

      // The normal vector from currentPoint to the other edge's line
      Vector3 normalVector =
	difference - otherEdgeDir * dot(difference, otherEdgeDir);
      double normalLength = length(normalVector);

      double intersectionParamThisEdge; // 0 - 1
      if (!equals(normalLength, 0.0, epsilon)) {
	intersectionParamThisEdge =
	  normalLength*normalLength / (dot(normalVector, thisEdge));
      }
      else {
	intersectionParamThisEdge = 0.0;
      }

      if (intersectionParamThisEdge > -epsilon &&
	  (intersectionParamThisEdge < 1.0 + epsilon)) {
	Vector3 intersectionPoint = currentPoint + thisEdge *
	  intersectionParamThisEdge;

	double intersectionParamOtherEdge;
	intersectionParamOtherEdge = dot(otherEdgeDir, intersectionPoint - polygon2[j]) / otherEdgeLength;

	if (intersectionParamOtherEdge > -epsilon &&
	    intersectionParamOtherEdge < 1.0 + epsilon) {
	  interestPoints.push_back(intersectionPoint);
	}
      }
    }
  }

  std::vector<Vector3> newInterestPoints;
  // Eliminate redundant interest points
  for (unsigned int i = 0; i < interestPoints.size(); i++) {
    bool keepThis = true;
    for (unsigned int j = i+1; j < interestPoints.size(); j++) {
      if (vequals(interestPoints[i], interestPoints[j], epsilon)) {
	keepThis = false;
	break;
      }
    }
    if (keepThis) {
      newInterestPoints.push_back(interestPoints[i]);
    }
  }

  if (newInterestPoints.size() < 4)
    return newInterestPoints;

  //Compute convex hull
  //Find rightmost point
  double maxX = -FLT_MAX;
  unsigned int startPoint;
  for (unsigned int i = 0; i < newInterestPoints.size(); i++) {
    if (newInterestPoints[i].x > maxX) {
      startPoint = i;
      maxX = newInterestPoints[i].x;
    }
  }

  unsigned int currentPoint = startPoint;

  std::vector<Vector3> outPolygon;
  do {
    outPolygon.push_back(newInterestPoints[currentPoint]);
    unsigned int nextPoint = (currentPoint == newInterestPoints.size()-1 ?
	0 : currentPoint + 1);
    
    Vector3 edge = newInterestPoints[nextPoint] - 
      newInterestPoints[currentPoint];
    for (unsigned int otherPoint = 0; otherPoint < newInterestPoints.size(); otherPoint++) {
      if (otherPoint != currentPoint && otherPoint != nextPoint) {
	double leftness = cross(edge, newInterestPoints[otherPoint] -
	    newInterestPoints[currentPoint]).z;
	if (leftness < 0.0) {
	  nextPoint = otherPoint;
	  edge = newInterestPoints[otherPoint] - newInterestPoints[currentPoint];
	}
      }
    }

    currentPoint = nextPoint;
  } while (currentPoint != startPoint);

  return outPolygon;
}


//std::vector<Vector3>
//findPolygonIntersection(const std::vector<Vector3> &polygon1, 
//    const std::vector<Vector3> &polygon2)
//{
//  std::vector<Vector3> outPolygon;
//
//  bool someIntersectionFound = false;
//
//  int currentFollowedPolygon = 1;
//
//  int currentIndex = 0;
//  Vector3 currentPoint = polygon1[currentIndex]; // Start point of current edge
//  Vector3 nextPoint = polygon1[currentIndex+1]; // End point of current edge
//  unsigned int nextIndex = 1; // Index that will be used for next start point unless
//  // an intersection is found
//#ifdef DEBUG
//  Vector3 norm1 = cross(polygon1[1]-polygon1[0], polygon1.back()-polygon1[0]);
//  normalise(norm1);
//  Vector3 norm2 = cross(polygon2[1]-polygon2[0], polygon2.back()-polygon2[0]);
//  normalise(norm2);
//  if (!vequals(norm1, norm2, 0.001)) {
//    // log("Comparing non-coplanar polygons!");
//    return 0.0;
//  }
//#endif
//
//  while (outPolygon.empty() || !vequals(currentPoint, outPolygon[0], 0.001))
//  {
////    cout << "(" << currentPoint.x << ", " << currentPoint.y << ") -> (" <<
////      nextPoint.x << ", " << nextPoint.y << ") follwing " << currentFollowedPolygon << endl;
//    Vector3 thisEdge = nextPoint - currentPoint;
//    double thisEdgeLength = length(thisEdge);
//    Vector3 thisEdgeDir = thisEdge / thisEdgeLength;
//
//    double smallestIntersectionParameter = 1.0;
//    int foundIntersection = 0;
//    Vector3 intersectionPoint;
//    int intersectionIndex;
//
//    unsigned int otherPolygonSize =
//      currentFollowedPolygon == 1 ? polygon2.size() : polygon1.size();
//
//    for (unsigned int i = 0; i < otherPolygonSize; i++) {
//      unsigned int iplus = (i == otherPolygonSize-1 ? 0 : i + 1);
//
//      Vector3 difference;
//      Vector3 otherEdgeDir;
//      double otherEdgeLength;
//      if (currentFollowedPolygon == 1) {
//	difference = polygon2[i] - currentPoint;
//	otherEdgeDir = polygon2[iplus] - polygon2[i];
//      } 
//      else {
//	difference = polygon1[i] - currentPoint;
//	otherEdgeDir = polygon1[iplus] - polygon1[i];
//      }
//
//      if (abs(dot(thisEdge, otherEdgeDir)) == 1.0) {
//	continue;
//      }
//
//      otherEdgeLength = length(otherEdgeDir);
//      otherEdgeDir /= otherEdgeLength;
//
//      // The normal vector from currentPoint to the other edge's line
//      Vector3 normalVector =
//	difference - otherEdgeDir * dot(difference, otherEdgeDir);
//      double normalLength = length(normalVector);
//
//      double intersectionParamThisEdge; // 0 - 1
//      if (!equals(normalLength, 0.0, 0.001)) {
//	intersectionParamThisEdge =
//	  normalLength*normalLength / (dot(normalVector, thisEdge));
//      }
//      else {
//	intersectionParamThisEdge = 0.0;
//      }
//
//      if (intersectionParamThisEdge > 0.0 &&
//	  (intersectionParamThisEdge < smallestIntersectionParameter ||
//	  equals(intersectionParamThisEdge, 1.0, 0.001))) {
//	intersectionPoint = currentPoint + thisEdge *
//	  intersectionParamThisEdge;
//
//	double intersectionParamOtherEdge;
//	if (currentFollowedPolygon == 1) {
//	  intersectionParamOtherEdge = dot(otherEdgeDir, intersectionPoint - polygon2[i]) / otherEdgeLength;
//	}
//	else {
//	  intersectionParamOtherEdge = dot(otherEdgeDir, intersectionPoint - polygon1[i]) / otherEdgeLength;
//	}
//
//	if (intersectionParamOtherEdge >= 0.0 &&
//	    intersectionParamOtherEdge <= 1.0) {
//
//	  // It's an intersection in fact!
//	  smallestIntersectionParameter = intersectionParamThisEdge;
//
//	  // Check winding of intersection.
//	  if (cross(thisEdgeDir, otherEdgeDir).z > 0.0) {
//	    // We're passing an edge that is going left
//	    // meaning we're leaving the polygon.
//
//	    foundIntersection = 1;
//	    intersectionIndex = iplus;
//	  }
//	  else {
//	    // We're passing an edge that is going right
//	    // meaning we are entering the other polgyon,
//	    // meaning we were outside the other polygon.
//
//	    foundIntersection = 2;
//	  }
//	}
//      }
//    }
//
//    if (foundIntersection == 0) {
//      outPolygon.push_back(currentPoint);
//      currentPoint = nextPoint;
//      nextIndex++;
//
//      if (currentFollowedPolygon == 1) {
//	if (nextIndex == polygon1.size())
//	  nextIndex = 0;
//	nextPoint = polygon1[nextIndex];
//      }
//      else {
//	if (nextIndex == polygon2.size())
//	  nextIndex = 0;
//	nextPoint = polygon2[nextIndex];
//      }
//    }
//    else if (foundIntersection == 1) {
//      // Store the point, and start following the other polygon instead
//
//      outPolygon.push_back(currentPoint);
//      currentFollowedPolygon = 3 - currentFollowedPolygon;
//
//      currentPoint = intersectionPoint;
//      nextIndex = intersectionIndex;
//      nextPoint = currentFollowedPolygon == 1 ?
//	polygon1[nextIndex] :
//	polygon2[nextIndex];
//
//      someIntersectionFound = true;
//    }
//    else {
//      // Clear all stored points, but keep following the
//      // current polygon (since the other polygon edge is
//      // leaving the current polygon at this point!)
//
//      outPolygon.clear();
//      currentPoint = intersectionPoint;
//      //	      nextIndex = nextIndex;
//      //	      nextPoint = nextPoint;
//
//      someIntersectionFound = true;
//    }
//  }
//
//  if (someIntersectionFound) {
//    return outPolygon;
//  }
//
//  // The polygons don't intersect! Find out if either is
//  // entirely enclosed by the other
//  bool polygon2Inside1 = true;
//  for (unsigned int i = 0; i < polygon1.size(); i++) {
//    unsigned int iplus = (i == polygon1.size()-1 ? 0 : i + 1);
//
//    if (cross(polygon1[iplus]-polygon1[i], polygon2[0]-polygon1[i]).z < 0.0) {
//      polygon2Inside1 = false;
//      break;
//    }
//  }
//
//  if (polygon2Inside1) {
//    return polygon2;
//  }
//
//  bool polygon1Inside2 = true;
//  for (unsigned int i = 0; i < polygon2.size(); i++) {
//    unsigned int iplus = (i == polygon2.size()-1 ? 0 : i + 1);
//
//    if (cross(polygon2[iplus]-polygon2[i], polygon1[0]-polygon2[i]).z < 0.0) {
//      polygon1Inside2 = false;
//      break;
//    }
//  }
//
//  if (polygon1Inside2) {
//    return polygon1;
//  }
//
//  outPolygon.clear();
//  return outPolygon;
//}

double
findOverlappingArea(const std::vector<Vector3>& polygon, Vector3 circleCenter, double circleRadius, const Vector3 &circleNormal)
{
  const Vector3 zeroVec = vector3(0,0,0);
  unsigned nextIndex = 1;
  vector<Vector3> entryEgressPoints;
  vector<Vector3> internalPolygonPoints;
  bool entryEgressPointsStartWithEntry = false;
  bool inside = true;
  Vector3 currentPoint = polygon[0];
  bool finished = false;
  while (!finished) {
    Vector3 nextPoint = polygon[nextIndex];
    Vector3 radiusVector = (currentPoint-circleCenter);

    if (nextPoint == currentPoint) {
      nextIndex++;
      if (nextIndex >= polygon.size())
	nextIndex = 0;
      continue;
    }

    Vector3 edge = nextPoint - currentPoint;
    double edgeLength = length(edge);
    Vector3 edgeDir = edge / edgeLength;

    Vector3 intersection1;
    Vector3 intersection2;

    Vector3 shortestRadius = radiusVector - edgeDir*dot(edgeDir, radiusVector);
    double shortestRadiusLength = length(shortestRadius);

    if (shortestRadiusLength <= circleRadius) {
      Vector3 halfChord = edgeDir * sqrt(circleRadius*circleRadius - 
	  shortestRadiusLength*shortestRadiusLength);

      intersection1 = circleCenter+shortestRadius-halfChord;
      intersection2 = circleCenter+shortestRadius+halfChord;

      double paramOfIntersection1 = dot(edgeDir, intersection1-currentPoint);
      double paramOfIntersection2 = dot(edgeDir, intersection2-currentPoint);

      if (paramOfIntersection1 == 0.0 && paramOfIntersection2 > 0.0) {
	// Edge is entering circle at currentPoint.
	// This will already have registered at the end point of the
	// last edge, but if that edge was internal, we need to reverse
	// the egress point created then.
	if (!inside) {
	  entryEgressPoints.pop_back();
	  if (entryEgressPoints.empty()) {
	    // Need to reset this too
	    entryEgressPointsStartWithEntry = false;
	  }
	  inside = true;
	}
      }

      if (paramOfIntersection1 == edgeLength) {
	// Edge is entering the circle at nextPoint
	if (entryEgressPoints.size() == 0) {
	  entryEgressPointsStartWithEntry = true;
	  internalPolygonPoints.clear(); //May have added points under the assumption
	  // that we were inside the circle
	}
	entryEgressPoints.push_back(nextPoint);
	internalPolygonPoints.push_back(nextPoint);
	currentPoint = nextPoint;

	inside = true;

	if (nextIndex == 0)
	  finished = true;
	nextIndex++;
	if (nextIndex >= polygon.size())
	  nextIndex = 0;
      }
      else if (paramOfIntersection2 == edgeLength) {
	// Edge is leaving the circle at nextPoint 
	entryEgressPoints.push_back(nextPoint);
	internalPolygonPoints.push_back(nextPoint);
	inside = false;

	currentPoint = nextPoint;

	if (nextIndex == 0)
	  finished = true;
	nextIndex++;
	if (nextIndex >= polygon.size())
	  nextIndex = 0;
      }
      else if (paramOfIntersection1 == paramOfIntersection2) {
	// Edge is tangent to circle except at the end point.
	// Ignore intersection, treat as normal edge
	if (inside) {
	  internalPolygonPoints.push_back(nextPoint);
	}
	currentPoint = nextPoint;

	if (nextIndex == 0)
	  finished = true;
	nextIndex++;
	if (nextIndex >= polygon.size())
	  nextIndex = 0;
      }
      else if (paramOfIntersection1 > 0.0 && paramOfIntersection1 < edgeLength) {
	// Edge is entering the circle at intersection1 between endpoints
	if (entryEgressPoints.size() == 0) {
	  entryEgressPointsStartWithEntry = true;
	  internalPolygonPoints.clear(); //May have added points under the assumption
	  // that we were inside the circle
	}
	entryEgressPoints.push_back(intersection1);
	internalPolygonPoints.push_back(intersection1);
	currentPoint = intersection1;

	inside = true;
      }
      else if (paramOfIntersection2 > 0.0 && paramOfIntersection2 < edgeLength) {
	// Edge is leaving the circle at intersection2 between endpoints
	entryEgressPoints.push_back(intersection2);
	internalPolygonPoints.push_back(intersection2);
	inside = false;

	currentPoint = nextPoint;

	if (nextIndex == 0)
	  finished = true;
	nextIndex++;
	if (nextIndex >= polygon.size())
	  nextIndex = 0;
      }
      else {
	// Edge is not intersecting the circle. If it is inside the circle,
	// add the endpoint
	if (inside) {
	  internalPolygonPoints.push_back(nextPoint);
	}
	currentPoint = nextPoint;

	if (nextIndex == 0)
	  finished = true;
	nextIndex++;
	if (nextIndex >= polygon.size())
	  nextIndex = 0;
      }
    }
    else {
      // Line entirely outside circle. just move on to the next
      // point.

      currentPoint = nextPoint;

      if (nextIndex == 0)
	finished = true;
      nextIndex++;
      if (nextIndex >= polygon.size())
	nextIndex = 0;
    }
  }

  // Add up area of polygon and circle arc segments
  double totalArea = getPolygonArea(internalPolygonPoints);

  // Segments start egress points and end at entry points
  // (i.e. egress of the polygon edge is entry of the circle and the
  // beginning of an overlapping segment)
  if (entryEgressPoints.size() > 1) { // There may be 1 invalid egress point
    // left in the vector from an internal-to-internal degenerate vertex
    unsigned int entryIndex;
    unsigned int egressIndex;
    if (entryEgressPointsStartWithEntry) {
      entryIndex = 0;
      egressIndex = entryEgressPoints.size()-1;
    }
    else {
      entryIndex = 1;
      egressIndex = 0;
    }
    while (entryIndex <= entryEgressPoints.size()-1) {
      const Vector3& entryPoint = entryEgressPoints[entryIndex];
      const Vector3& egressPoint = entryEgressPoints[egressIndex];

      double entryAngle = atan2((entryPoint-circleCenter).y/circleRadius,
	  (entryPoint-circleCenter).x/circleRadius);
      double egressAngle = atan2((egressPoint-circleCenter).y/circleRadius,
	  (egressPoint-circleCenter).x/circleRadius);

      double theta = entryAngle - egressAngle;
      if (theta < 0.0) theta += M_PI*2;
      if (theta > M_PI*2) theta -= M_PI*2;

      totalArea += circleRadius*circleRadius * 
	(theta/2 - sin(theta/2)*cos(theta/2)); // Should work even for segments
      //longer than half a circle


      entryIndex += 2;
      egressIndex = entryIndex - 1;
    }
    return totalArea;
  }
  else {
    // Circle is either completely outside or completely inside polygon.
    // Checking one vertex is sufficient.
    if (length(polygon[0]-circleCenter) <= circleRadius) {
      // Polygon is inside circle
      return getPolygonArea(polygon);
    }
    else {
      // Circle is inside polygon
      return M_PI*circleRadius*circleRadius;
    }
  }
}

double
getPolygonArea(const std::vector<Vector3> &polygon)
{
  double ret = 0.0;
//  for (unsigned int i = 0; i < polygon.size(); i++) {
//    unsigned int iplus = (i == polygon.size()-1) ? 0 : i + 1;
//    ret += polygon[i].x*polygon[iplus].y - polygon[i].y*polygon[iplus].x;
//  }
  if (polygon.size() < 3) {
    return 0.0;
  }

  Vector3 normal = cross(polygon[1]-polygon[0], polygon.back()-polygon[0]);
  normalise(normal);
  for (unsigned int i = 0; i < polygon.size(); i++) {
    unsigned int iplus = (i == polygon.size()-1) ? 0 : i + 1;
    ret += dot(normal, cross(polygon[i], polygon[iplus]));
  }
  return 0.5*ret;
}

double
getMaxPolygonClearance(const std::vector<Vector3> &polygon) 
{
  //Find all bisectors
  std::vector<Vector3>bisectors;
  std::vector<Vector3>edgeNormals;
  bisectors.reserve(polygon.size());
  edgeNormals.reserve(polygon.size());
  Vector3 up = cross(polygon[1] - polygon[0], polygon.back() - polygon[0]);
  normalise(up);

  Vector3 lastEdgeDir = polygon.back()-polygon[0];
  normalise(lastEdgeDir);

  for (unsigned int i = 0; i < polygon.size(); i++) {
    unsigned int iplus = (i == polygon.size()-1) ? 0 : i + 1;
    Vector3 currentEdgeDir = polygon[iplus]-polygon[i];
    normalise(currentEdgeDir);
    edgeNormals.push_back(cross(up, currentEdgeDir));

    Vector3 bisectorDir = currentEdgeDir + lastEdgeDir;
    normalise(bisectorDir);
    bisectors.push_back(bisectorDir);

    lastEdgeDir = -currentEdgeDir;
  }

  double maxDistance = 0.0;
  //Find all intersections between bisectors
  for (unsigned int i = 0; i < polygon.size(); i++) {
    for (unsigned int j = i+1; j < polygon.size(); j++) {
      Vector3 difference = polygon[j] - polygon[i];

      Vector3 normalVector =
	difference - bisectors[j] * dot(difference, bisectors[j]);
      double normalLength = length(normalVector);
      double intersectionParam =  // 0 - inf
	normalLength*normalLength / (dot(normalVector, bisectors[i]));
      double intersectionNormalDistance = 
	intersectionParam * dot(edgeNormals[i], bisectors[i]);
      if (intersectionNormalDistance > 0.0 &&
	  intersectionNormalDistance > maxDistance) {
	Vector3 intersectionPoint = polygon[i] + bisectors[i]*intersectionParam;
	double otherParameter = dot(intersectionPoint - polygon[j], bisectors[j]);
	if (otherParameter > 0.0) {
	  if (!equals(otherParameter, intersectionParam, 0.001)) {
	    cout << "Error! Something not right!";
	    return 0.0;
	  }
	  maxDistance = intersectionNormalDistance;
	}
      }
    }
  }
  return maxDistance;
}

void
sampleOnnessDistribution(const Object *objectS, Object *objectO,
    std::vector<Vector3> &outPoints, double xmin, double xmax, 
    double ymin, double ymax,  
    double zmin, double zmax,
    double startStep, double minStep) {
  double threshold = 0.5;
  std::set<Vector3> cloudPoints;
  fromAngleAxis(objectO->pose.rot, 0.0, vector3(0.0, 0.0, 1.0));

  double step = startStep;

  for (double x = xmin; x <= xmax; x += step) {
    for (double y = ymin; y <= ymax; y += step) {
      for (double z = zmin; z <= zmax; z += step) {
	objectO->pose.pos = vector3(x, y, z);
	double onness = evaluateOnness(objectS, objectO);
	if (onness > threshold) {
	  cloudPoints.insert(objectO->pose.pos);
	}
      }
    }
  }

  outPoints.clear();

  std::set<Vector3> shellPoints;
  for (std::set<Vector3>::iterator it = cloudPoints.begin();
      it != cloudPoints.end(); it++) {
    Vector3 neigh1 = vector3(it->x-step, it->y, it->z); 
    Vector3 neigh2 = vector3(it->x+step, it->y, it->z); 
    Vector3 neigh3 = vector3(it->x, it->y-step, it->z); 
    Vector3 neigh4 = vector3(it->x, it->y+step, it->z); 
    Vector3 neigh5 = vector3(it->x, it->y, it->z-step); 
    Vector3 neigh6 = vector3(it->x, it->y, it->z+step); 
    if (cloudPoints.find(neigh1) == cloudPoints.end() ||
	cloudPoints.find(neigh2) == cloudPoints.end() ||
	cloudPoints.find(neigh3) == cloudPoints.end() ||
	cloudPoints.find(neigh4) == cloudPoints.end() ||
	cloudPoints.find(neigh5) == cloudPoints.end() ||
	cloudPoints.find(neigh6) == cloudPoints.end()) {
      shellPoints.insert(*it);
    }
  }
  while (step >= minStep * 2.0) {
    step /= 2.0;
    std::set<Vector3> tmpPoints = shellPoints;
    for (std::set<Vector3>::iterator it = shellPoints.begin();
	it != shellPoints.end(); it++) {
      for (double x = it->x-step; x <= it->x+step; x+=step) {
	for (double y = it->y-step; y <= it->y+step; y+=step) {
	  for (double z = it->z-step; z <= it->z+step; z+=step) {
	    Vector3 candidate = vector3(x, y, z);
	    if (cloudPoints.find(candidate) == cloudPoints.end()) {
	      objectO->pose.pos = candidate;
	      double onness = evaluateOnness(objectS, objectO);
	      if (onness > threshold) {
		cloudPoints.insert(candidate);
		tmpPoints.insert(candidate);
	      }	      
	    }
	    else {
	      tmpPoints.insert(candidate);
	    }
	  }
	}
      }
    }

    shellPoints.clear();
    for (std::set<Vector3>::iterator it = tmpPoints.begin();
	it != tmpPoints.end(); it++) {
      Vector3 neigh[6] = {vector3(it->x-step, it->y, it->z), 
	vector3(it->x+step, it->y, it->z), 
	vector3(it->x, it->y-step, it->z), 
	vector3(it->x, it->y+step, it->z), 
	vector3(it->x, it->y, it->z-step), 
	vector3(it->x, it->y, it->z+step)}; 
      bool isShell = false;
      for (int i = 0; i < 6; i++) {
	if (cloudPoints.find(neigh[i]) == cloudPoints.end()) {
	  objectO->pose.pos = neigh[i];
	  double onness = evaluateOnness(objectS, objectO);
	  if (onness < threshold) {
	    isShell = true;
	  }
	}
      }
      if (isShell) {
	shellPoints.insert(*it);
      }
    }
  };

  outPoints.insert(outPoints.end(), shellPoints.begin(), shellPoints.end());
}
};
