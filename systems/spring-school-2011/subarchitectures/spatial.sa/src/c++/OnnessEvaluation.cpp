#include <cast/core/CASTUtils.hpp>
#include "RelationEvaluation.hpp"
using namespace cast;
#include <Pose3.h>
#include <iostream>
#include <set>
  
using namespace std;
using namespace cogx::Math;


namespace spatial {

const double planeThickness = 0.05;

double
evaluateOnness(const Object *objectS, const Object *objectO)
{
  // Retrieve pose for supporting object (S)
  Pose3 Spose = objectS->pose;
  // Retrieve model for S
  // Retrieve pose for supported object (O)
  Pose3 Opose = objectO->pose;
  // Retrieve model for O

  //int supportSurfaceType; // 0 for point, 1 for line segment, 2 for circle, 3 for polygon
  //Vector3 supportSurfaceCenter; 		// For 0 and 2, epicenter
  //double supportSurfaceRadius;
  std::vector<Vector3> supportSurfaceVertices; // For 1 and 3, vertices
  //Vector3 supportPlaneNormal;
  //double supportPlaneOffset; // dot product of plane point and plane normal

  // Find topmost surface of S:
  //	if S is a plane object, surface is a polygon or circle
  //	if S is a box, surface is the rectangle whose normal's Z-component
  //		is largest.
  //	if S is a sphere, surface is a point: the projection of the 
  //		sphere's center in O's bottom surface (defer)
  //	if S is a cylinder, surface is a circle if either end's Z-component
  //		exceeds sqrt(0.5), else it is a line segment, namely
  //		the projection of the central axis on O's bottom surface


  Witness witness;
  vector<Vector3> patch;
  double maxPatchClearance;

  if ((objectS->type == OBJECT_BOX || objectS->type == OBJECT_HOLLOW_BOX) &&
      (objectO->type == OBJECT_BOX || objectO->type == OBJECT_HOLLOW_BOX)) {
    witness = findContactPatch(*((BoxObject*)objectS), *((BoxObject*)objectO),
	&patch, &maxPatchClearance);
  }

  else if (objectS->type == OBJECT_PLANE &&
      (objectO->type == OBJECT_BOX || objectO->type == OBJECT_HOLLOW_BOX)) {

    BoxObject *Obox = (BoxObject*)objectO;
    PlaneObject *Splane = (PlaneObject*)objectS;
    if (Splane->shape == PLANE_OBJECT_RECTANGLE) {
      BoxObject Sbox;
      Sbox.type = OBJECT_BOX;
      Sbox.radius1 = Splane->radius1;
      Sbox.radius2 = Splane->radius2;
      Sbox.radius3 = planeThickness*0.5;
      Sbox.pose = Splane->pose;
      Sbox.pose.pos.z -= planeThickness*0.5;
      witness = findContactPatch(Sbox, *Obox, &patch, &maxPatchClearance);
    }
  }

  normalise(witness.normal);

  double contactOnness = 0.0;
  // Put patch and pose in one plane
  if (witness.normal.z > 0) {
    contactOnness = witness.normal.z;
//    contactOnness = contactOnness*contactOnness;
  }
  //	else {
  //	  vector<Vector3> tmpPatch(patch.size());
  //	  for (unsigned int i = 0; i < patch.size(); i++) {
  //	    tmpPatch[patch.size()-i-1].z = Obox->pose.pos.z;
  //	  }
  //	  std::copy(tmpPatch.begin(), tmpPatch.end(), patch.begin());
  //	}

  if (contactOnness > 0.0) {
    double COMDistance;
    if (patch.size() > 2) {
    for (unsigned int i = 0; i < patch.size(); i++) {
      patch[i].z = objectO->pose.pos.z;
    }
    COMDistance = getDistanceToPolygon(objectO->pose.pos, 
	patch);
    }
    else {
      //No patch; just take the horizontal distance to the
      //closest point
      //NOTE: this is not really the shortest distance from the
      //COM to an "almost-patch" - but the distance penalty
      //should hopefully make sure any discontinuity isn't too
      //jarring.

      Vector3 tmp = objectO->pose.pos - witness.point1;
      tmp.z = 0;
      COMDistance = length(tmp);
    }
    
    double distanceFactor = COMDistance / maxPatchClearance;

    // Formula for onness based on COMDistance:
    // y = (1+exp(B*(-1-C))/(1+exp(B*(x-C)))
    // where x is the distanceFactor,
    // B is a steepness coefficient 0<B<inf
    // and C is an offset -l_max <= C < inf

    contactOnness *= 
      (1 + exp(supportCOMContainmentSteepness*(-1-supportCOMContainmentOffset)))
      		/ (1 + exp(supportCOMContainmentSteepness*
		(distanceFactor-supportCOMContainmentOffset)));
//    if (COMDistance > 0.0) {
//      contactOnness /= (1+COMDistance/supportCOMContainmentSteepness);
//    }
  }

  double distanceOnness;
  if (witness.distance > 0.0) {
//    distanceOnness = witness.distance/distanceFalloffOutside;
    distanceOnness = exp(-witness.distance/distanceFalloffOutside * 0.3010129996);
  }
  else {
//    distanceOnness = witness.distance/distanceFalloffInside;
    distanceOnness = exp(witness.distance/distanceFalloffInside * 0.3010129996);
  }
//  distanceOnness = 1/(1+distanceOnness*distanceOnness);

  return min(distanceOnness, contactOnness);

  // Old version onness below here!

  /*
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
  */
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

Witness
findContactPatch(const BoxObject &boxA, const BoxObject &boxB, vector<Vector3> *outPatch, double *maxPatchClearance)
{
  const int edges[] = {0,1, 1,2, 2,3, 3,0, 0,4, 4,5, 5,1, 5,6,
    6,2, 6,7, 7,3, 7,4}; //pairs of ints, indexing into BVertices
  //Transform B into local coordinate system of A for ease of conceptualization
  Pose3 BposeInA;
  transformInverse(boxA.pose, boxB.pose, BposeInA); 
  Pose3 AposeInB;
  transformInverse(boxB.pose, boxA.pose, AposeInB); 

  const Vector3 BVertices[] = {{boxB.radius1, boxB.radius2, boxB.radius3},
    {-boxB.radius1, boxB.radius2, boxB.radius3},
    {-boxB.radius1, -boxB.radius2, boxB.radius3},
    {boxB.radius1, -boxB.radius2, boxB.radius3},
    {boxB.radius1, boxB.radius2, -boxB.radius3},
    {-boxB.radius1, boxB.radius2, -boxB.radius3},
    {-boxB.radius1, -boxB.radius2, -boxB.radius3},
    {boxB.radius1, -boxB.radius2, -boxB.radius3}};
  const Vector3 AVertices[] = {{boxA.radius1, boxA.radius2, boxA.radius3},
    {-boxA.radius1, boxA.radius2, boxA.radius3},
    {-boxA.radius1, -boxA.radius2, boxA.radius3},
    {boxA.radius1, -boxA.radius2, boxA.radius3},
    {boxA.radius1, boxA.radius2, -boxA.radius3},
    {-boxA.radius1, boxA.radius2, -boxA.radius3},
    {-boxA.radius1, -boxA.radius2, -boxA.radius3},
    {boxA.radius1, -boxA.radius2, -boxA.radius3}};
  Vector3 AVerticesInB[8];
  Vector3 BVerticesInA[8];
  for (int i = 0; i < 8; i++) {
    BVerticesInA[i] = transform(BposeInA, BVertices[i]);
    AVerticesInB[i] = transform(AposeInB, AVertices[i]);
  }

  double wr = boxA.radius1;
  double dr = boxA.radius2;
  double hr = boxA.radius3;

  bool intersecting = isIntersecting(wr, dr, hr, BVerticesInA);

  vector<Vector3> BEdgesInA;
  BEdgesInA.reserve(12);
  vector<Vector3> AEdgesInB;
  AEdgesInB.reserve(12);
  for (int edgeNo = 0; edgeNo < 12; edgeNo++) {
    Vector3 &point1 = BVerticesInA[edges[edgeNo*2]];
    Vector3 &point2 = BVerticesInA[edges[edgeNo*2+1]];
    Vector3 edge = point2 - point1; 
    BEdgesInA.push_back(edge);

    Vector3 &point3 = AVerticesInB[edges[edgeNo*2]];
    Vector3 &point4 = AVerticesInB[edges[edgeNo*2+1]];
    edge = point4 - point3; 
    AEdgesInB.push_back(edge);
  }

  vector<Witness> cornerWitnessesBInA;
  getCornerWitnesses(wr, dr, hr, BVerticesInA, BEdgesInA, cornerWitnessesBInA);

  vector<Witness> edgeWitnesses;
  getEdgeWitnesses(wr, dr, hr, BVerticesInA, BEdgesInA, edgeWitnesses, intersecting);

  wr = boxB.radius1;
  dr = boxB.radius2;
  hr = boxB.radius3;

  vector<Witness> cornerWitnessesAInB;
  getCornerWitnesses(wr, dr, hr, AVerticesInB, AEdgesInB, cornerWitnessesAInB);

  Witness bestWitness;

  //True iff point2 and idOnB and typeOnB in bestWitness
  //actually refer to A
  bool AisB = false; 

  if (intersecting) {
    bestWitness.distance = -FLT_MAX;
    bool found = false;
    wr = boxA.radius1;
    dr = boxA.radius2;
    hr = boxA.radius3;
    for(unsigned int i = 0; i < cornerWitnessesBInA.size(); i++) {
      if (cornerWitnessesBInA[i].distance > bestWitness.distance &&
	  cornerWitnessesBInA[i].point1.x <= wr &&
	  cornerWitnessesBInA[i].point1.x >= -wr &&
	  cornerWitnessesBInA[i].point1.y <= dr &&
	  cornerWitnessesBInA[i].point1.y >= -dr &&
	  cornerWitnessesBInA[i].point1.z <= hr &&
	  cornerWitnessesBInA[i].point1.z >= -hr) {
	AisB = false;
	bestWitness = cornerWitnessesBInA[i];
	bestWitness.point1 = transform(boxA.pose, bestWitness.point1);
	bestWitness.point2 = transform(boxA.pose, bestWitness.point2);
	bestWitness.normal = boxA.pose.rot * bestWitness.normal;

	found = true;
      }
    }
    wr = boxB.radius1;
    dr = boxB.radius2;
    hr = boxB.radius3;
    for(unsigned int i = 0; i < cornerWitnessesAInB.size(); i++) {
      if (cornerWitnessesAInB[i].distance > bestWitness.distance &&
	  cornerWitnessesAInB[i].point1.x <= wr &&
	  cornerWitnessesAInB[i].point1.x >= -wr &&
	  cornerWitnessesAInB[i].point1.y <= dr &&
	  cornerWitnessesAInB[i].point1.y >= -dr &&
	  cornerWitnessesAInB[i].point1.z <= hr &&
	  cornerWitnessesAInB[i].point1.z >= -hr) {
	AisB = true;
	bestWitness = cornerWitnessesAInB[i];
	bestWitness.point1 = transform(boxB.pose, bestWitness.point1);
	bestWitness.point2 = transform(boxB.pose, bestWitness.point2);
	bestWitness.normal = boxB.pose.rot * bestWitness.normal;

	found = true;
      }
    }
    for(unsigned int i = 0; i < edgeWitnesses.size(); i++) {
      if (edgeWitnesses[i].distance > bestWitness.distance) {
	AisB = false;
	bestWitness = edgeWitnesses[i];
	bestWitness.point1 = transform(boxA.pose, bestWitness.point1);
	bestWitness.point2 = transform(boxA.pose, bestWitness.point2);
	bestWitness.normal = boxA.pose.rot * bestWitness.normal;

	found = true;
      }
    }
  }


  else {
    //Find the witness points that are closest.
    //If the closest witness point is vertex-face, and the face point is not actually
    //on the face, the closest distance is a vertex-edge or a vertex-vertex case.
    //If an edge-edge witness pair, and one of the witness points isn't on the edge,
    //it's a vertex-edge case; if neither is, it's a vertex-vertex case.
    bestWitness.distance = FLT_MAX;
    wr = boxA.radius1;
    dr = boxA.radius2;
    hr = boxA.radius3;

    const int edgeOverflowX[] = {3,1, 4,6, -1,-1, 10,8, -1,-1, 11,7};
    const int edgeOverflowY[] = {0,2, -1,-1, 6,8, -1,-1, 4,10, 5,9};
    const int edgeOverflowZ[] = {-1,-1, 0,5, 1,7, 2,9, 3,11, -1,-1};

    vector<Witness> cornerEdgeWitnessesBInA;
    vector<Witness> cornerEdgeWitnessesAInB;

    for(unsigned int i = 0; i < cornerWitnessesBInA.size(); i++) {
      if (cornerWitnessesBInA[i].distance < bestWitness.distance &&
	  cornerWitnessesBInA[i].distance >= 0) {
	Witness nextBestWitness = cornerWitnessesBInA[i];

	// Check whether the vertex' projection is on the face

	// Indices of the edges that a face borders on in posivite/negative
	// direction of X, Y, Z
	int faceID = nextBestWitness.idOnA;
	if (nextBestWitness.point1.x >= wr &&
	    edgeOverflowX[faceID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowX[faceID*2];
	  nextBestWitness.point1.x = wr;
	}
	else if (nextBestWitness.point1.x <= -wr &&
	    edgeOverflowX[faceID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowX[faceID*2+1];
	  nextBestWitness.point1.x = -wr;
	}
	else if (nextBestWitness.point1.y >= dr &&
	    edgeOverflowY[faceID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowY[faceID*2];
	  nextBestWitness.point1.y = dr;
	}
	else if (nextBestWitness.point1.y <= -dr &&
	    edgeOverflowY[faceID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowY[faceID*2+1];
	  nextBestWitness.point1.y = -dr;
	}
	else if (nextBestWitness.point1.z >= hr &&
	    edgeOverflowZ[faceID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowZ[faceID*2];
	  nextBestWitness.point1.z = hr;
	}
	else if (nextBestWitness.point1.z <= -hr &&
	    edgeOverflowZ[faceID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowZ[faceID*2+1];
	  nextBestWitness.point1.z = -hr;
	}
	else {
	  AisB = false;
	  bestWitness = nextBestWitness; 
	  // Transform the points into global coordinates
	  bestWitness.point1 = transform(boxA.pose, bestWitness.point1);
	  bestWitness.point2 = transform(boxA.pose, bestWitness.point2);
	  bestWitness.normal = boxA.pose.rot * bestWitness.normal;
	  continue;
	}

	nextBestWitness.distance = length(nextBestWitness.point1 - nextBestWitness.point2);
	cornerEdgeWitnessesBInA.push_back(nextBestWitness);
      }
    }

    for(unsigned int i = 0; i < edgeWitnesses.size(); i++) {
      if (edgeWitnesses[i].distance < bestWitness.distance) {
	Witness nextBestWitness = edgeWitnesses[i];

	bool newDistance = false;
	if (nextBestWitness.paramOnA > 1.0) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = edges[nextBestWitness.idOnA*2+1];
	  nextBestWitness.point1 = AVertices[nextBestWitness.idOnA];
	  newDistance = true;
	}
	if (nextBestWitness.paramOnA < 0.0) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = edges[nextBestWitness.idOnA*2];
	  nextBestWitness.point1 = AVertices[nextBestWitness.idOnA];
	  newDistance = true;
	}
	if (nextBestWitness.paramOnB > 1.0) {
	  nextBestWitness.typeOnB = WITNESS_VERTEX;
	  nextBestWitness.idOnB = edges[nextBestWitness.idOnB*2+1];
	  nextBestWitness.point2 = BVerticesInA[nextBestWitness.idOnB];
	  newDistance = true;
	}
	if (nextBestWitness.paramOnB < 0.0) {
	  nextBestWitness.typeOnB = WITNESS_VERTEX;
	  nextBestWitness.idOnB = edges[nextBestWitness.idOnB*2];
	  nextBestWitness.point2 = BVerticesInA[nextBestWitness.idOnB];
	  newDistance = true;
	}

	if (newDistance) {
	  nextBestWitness.distance = length(nextBestWitness.point1 - nextBestWitness.point2);
	}

	if (nextBestWitness.distance >= 0.0 && (
	      !newDistance || nextBestWitness.distance < bestWitness.distance)) {
	  AisB = false;
	  bestWitness = nextBestWitness;
	  bestWitness.point1 = transform(boxA.pose, bestWitness.point1);
	  bestWitness.point2 = transform(boxA.pose, bestWitness.point2);
	  bestWitness.normal = boxA.pose.rot * bestWitness.normal;
	}

      }
    }

    // Along each edge, tells you which vertex the witness should snap to if it
    // exceeds the positive and negative extent of the box, respectively
    const int vertexOverflowX[]={0,1, -1,-1, 3,2, -1,-1, -1,-1, 4,5, -1,-1, -1,-1,
      -1,-1, 7,6, -1,-1, -1,-1};
    const int vertexOverflowY[]={-1,-1, 1,2, -1,-1, 0,3, -1,-1, -1,-1, -1,-1, 5,6,
      -1,-1, -1,-1, -1,-1, 4,7};
    const int vertexOverflowZ[]={-1,-1, -1,-1, -1,-1, -1,-1, 0,4, -1,-1, 1,5, -1,-1,
      2,6, -1,-1, 3,7, -1,-1};

    for (unsigned int i = 0; i < cornerEdgeWitnessesBInA.size(); i++) {
      if (cornerEdgeWitnessesBInA[i].distance < bestWitness.distance) {
	Witness nextBestWitness = cornerEdgeWitnessesBInA[i];

	int edgeID = nextBestWitness.idOnA;

	if (nextBestWitness.point1.x > wr &&
	    vertexOverflowX[edgeID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowX[edgeID*2];
	  nextBestWitness.point1.x = wr;
	}
	else if (nextBestWitness.point1.x < -wr &&
	    vertexOverflowX[edgeID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowX[edgeID*2+1];
	  nextBestWitness.point1.x = -wr;
	}
	else if (nextBestWitness.point1.y > dr &&
	    vertexOverflowY[edgeID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowY[edgeID*2];
	  nextBestWitness.point1.y = dr;
	}
	else if (nextBestWitness.point1.y < -dr &&
	    vertexOverflowY[edgeID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowY[edgeID*2+1];
	  nextBestWitness.point1.y = -dr;
	}
	else if (nextBestWitness.point1.z > hr &&
	    vertexOverflowZ[edgeID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowZ[edgeID*2];
	  nextBestWitness.point1.z = hr;
	}
	else if (nextBestWitness.point1.z < -hr &&
	    vertexOverflowZ[edgeID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowZ[edgeID*2+1];
	  nextBestWitness.point1.z = -hr;
	}
	else {
	  AisB = false;
	  bestWitness = nextBestWitness;
	  // Transform the points into global coordinates
	  bestWitness.point1 = transform(boxA.pose, bestWitness.point1);
	  bestWitness.point2 = transform(boxA.pose, bestWitness.point2);
	  bestWitness.normal = boxA.pose.rot * bestWitness.normal;
	  continue;
	}

	nextBestWitness.distance = length(nextBestWitness.point1 - nextBestWitness.point2);
	if (nextBestWitness.distance < bestWitness.distance)
	{
	  AisB = false;
	  bestWitness = nextBestWitness;
	  bestWitness.point1 = transform(boxA.pose, bestWitness.point1);
	  bestWitness.point2 = transform(boxA.pose, bestWitness.point2);
	  bestWitness.normal = boxA.pose.rot * bestWitness.normal;
	}
      }
    }

    wr = boxB.radius1;
    dr = boxB.radius2;
    hr = boxB.radius3;
    for(unsigned int i = 0; i < cornerWitnessesAInB.size(); i++) {
      if (cornerWitnessesAInB[i].distance < bestWitness.distance &&
	  cornerWitnessesAInB[i].distance >= 0) {
	Witness nextBestWitness = cornerWitnessesAInB[i];

	// Check whether the vertex' projection is on the face

	// Indices of the edges that a face borders on in posivite/negative
	// direction of X, Y, Z
	int faceID = nextBestWitness.idOnA;
	if (nextBestWitness.point1.x >= wr &&
	    edgeOverflowX[faceID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowX[faceID*2];
	  nextBestWitness.point1.x = wr;
	}
	else if (nextBestWitness.point1.x <= -wr &&
	    edgeOverflowX[faceID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowX[faceID*2+1];
	  nextBestWitness.point1.x = -wr;
	}
	else if (nextBestWitness.point1.y >= dr &&
	    edgeOverflowY[faceID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowY[faceID*2];
	  nextBestWitness.point1.y = dr;
	}
	else if (nextBestWitness.point1.y <= -dr &&
	    edgeOverflowY[faceID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowY[faceID*2+1];
	  nextBestWitness.point1.y = -dr;
	}
	else if (nextBestWitness.point1.z >= hr &&
	    edgeOverflowZ[faceID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowZ[faceID*2];
	  nextBestWitness.point1.z = hr;
	}
	else if (nextBestWitness.point1.z <= -hr &&
	    edgeOverflowZ[faceID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_EDGE;
	  nextBestWitness.idOnA = edgeOverflowZ[faceID*2+1];
	  nextBestWitness.point1.z = -hr;
	}
	else {
	  AisB = true;
	  if (nextBestWitness.distance >= 0) {
	    bestWitness = nextBestWitness; 
	    // Transform the points into global coordinates
	    bestWitness.point1 = transform(boxB.pose, bestWitness.point1);
	    bestWitness.point2 = transform(boxB.pose, bestWitness.point2);
	    bestWitness.normal = boxB.pose.rot * bestWitness.normal;
	  }
	  continue;
	}

	nextBestWitness.distance = length(nextBestWitness.point1 - nextBestWitness.point2);
	cornerEdgeWitnessesAInB.push_back(nextBestWitness);
      }
    }

    for (unsigned int i = 0; i < cornerEdgeWitnessesAInB.size(); i++) {
      if (cornerEdgeWitnessesAInB[i].distance < bestWitness.distance) {
	Witness nextBestWitness = cornerEdgeWitnessesAInB[i];

	int edgeID = nextBestWitness.idOnA;

	if (nextBestWitness.point1.x > wr &&
	    vertexOverflowX[edgeID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowX[edgeID*2];
	  nextBestWitness.point1.x = wr;
	}
	else if (nextBestWitness.point1.x < -wr &&
	    vertexOverflowX[edgeID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowX[edgeID*2+1];
	  nextBestWitness.point1.x = -wr;
	}
	else if (nextBestWitness.point1.y > dr &&
	    vertexOverflowY[edgeID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowY[edgeID*2];
	  nextBestWitness.point1.y = dr;
	}
	else if (nextBestWitness.point1.y < -dr &&
	    vertexOverflowY[edgeID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowY[edgeID*2+1];
	  nextBestWitness.point1.y = -dr;
	}
	else if (nextBestWitness.point1.z > hr &&
	    vertexOverflowZ[edgeID*2] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowZ[edgeID*2];
	  nextBestWitness.point1.z = hr;
	}
	else if (nextBestWitness.point1.z < -hr &&
	    vertexOverflowZ[edgeID*2+1] != -1) {
	  nextBestWitness.typeOnA = WITNESS_VERTEX;
	  nextBestWitness.idOnA = vertexOverflowZ[edgeID*2+1];
	  nextBestWitness.point1.z = -hr;
	}
	else {
	  AisB = true;
	  if (nextBestWitness.distance >= 0) {
	    bestWitness = nextBestWitness;
	    // Transform the points into global coordinates
	    bestWitness.point1 = transform(boxB.pose, bestWitness.point1);
	    bestWitness.point2 = transform(boxB.pose, bestWitness.point2);
	    bestWitness.normal = boxB.pose.rot * bestWitness.normal;
	  }
	  continue;
	}

	nextBestWitness.distance = length(nextBestWitness.point1 - nextBestWitness.point2);
	if (nextBestWitness.distance < bestWitness.distance && 
	    nextBestWitness.distance >= 0.0)
	{
	  bestWitness = nextBestWitness;
	  bestWitness.point1 = transform(boxB.pose, bestWitness.point1);
	  bestWitness.point2 = transform(boxB.pose, bestWitness.point2);
	  bestWitness.normal = boxB.pose.rot * bestWitness.normal;
	}
      }
    }
  }

  if (AisB) {
    Witness tmpWitness = bestWitness;
    bestWitness.typeOnA = bestWitness.typeOnB;
    bestWitness.idOnA = bestWitness.idOnB;
    bestWitness.point1 = bestWitness.point2;
    bestWitness.paramOnA = bestWitness.paramOnB;
    bestWitness.typeOnB = tmpWitness.typeOnA;
    bestWitness.idOnB = tmpWitness.idOnA;
    bestWitness.point2 = tmpWitness.point1;
    bestWitness.paramOnB = tmpWitness.paramOnA;
    bestWitness.normal = -bestWitness.normal;
  }

  if (outPatch != 0 || maxPatchClearance != 0) {
    const int VertexFaceNeigbors[] = {0,4,1, 0,1,2, 0,2,3, 0,3,4, 1,4,5, 2,1,5, 3,2,5, 4,3,5};
    const int EdgeFaceNeighbors[] = {0,1, 0,2, 0,3, 0,4, 1,4, 1,5, 1,2, 2,5, 2,3, 3,5, 3,4, 4,5};
    // Find the face on A and the face on B which 
    vector<int> facesOnA;
    vector<int> facesOnB;
    if (bestWitness.typeOnA == WITNESS_VERTEX) {
      facesOnA.push_back(VertexFaceNeigbors[bestWitness.idOnA*3]);
      facesOnA.push_back(VertexFaceNeigbors[bestWitness.idOnA*3+1]);
      facesOnA.push_back(VertexFaceNeigbors[bestWitness.idOnA*3+2]);
    }
    else if (bestWitness.typeOnA == WITNESS_EDGE) {
      facesOnA.push_back(EdgeFaceNeighbors[bestWitness.idOnA*2]);
      facesOnA.push_back(EdgeFaceNeighbors[bestWitness.idOnA*2+1]);
    }
    else {
      facesOnA.push_back(bestWitness.idOnA);
    }
    if (bestWitness.typeOnB == WITNESS_VERTEX) {
      facesOnB.push_back(VertexFaceNeigbors[bestWitness.idOnB*3]);
      facesOnB.push_back(VertexFaceNeigbors[bestWitness.idOnB*3+1]);
      facesOnB.push_back(VertexFaceNeigbors[bestWitness.idOnB*3+2]);
    }
    else if (bestWitness.typeOnB == WITNESS_EDGE) {
      facesOnB.push_back(EdgeFaceNeighbors[bestWitness.idOnB*2]);
      facesOnB.push_back(EdgeFaceNeighbors[bestWitness.idOnB*2+1]);
    }
    else {
      facesOnB.push_back(bestWitness.idOnB);
    }

    const Vector3 faceNormals[] = {vector3(0,0,1),
      vector3(0,1,0),
      vector3(-1,0,0),
      vector3(0,-1,0),
      vector3(1,0,0),
      vector3(0,0,-1)};

    double lowestDot = FLT_MAX;
    int bestFaceOnA;
    int bestFaceOnB;
    for (unsigned int i = 0; i < facesOnA.size(); i++) {
      for (unsigned int j = 0; j < facesOnB.size(); j++) {
	double normalDotProduct = dot(boxA.pose.rot * faceNormals[facesOnA[i]],
	    boxB.pose.rot * faceNormals[facesOnB[j]]);
	if (normalDotProduct < lowestDot) {
	  lowestDot = normalDotProduct;
	  bestFaceOnA = facesOnA[i];
	  bestFaceOnB = facesOnB[j];
	}
      }
    }


    const int FaceVertexNeighbors[] = {0,1,2,3, 0,4,5,1, 1,5,6,2, 2,6,7,3, 0,3,7,4, 4,7,6,5};

    if (maxPatchClearance != 0) {
      vector<Vector3> face;
      for (int i = 0; i < 4; i++) {
	face.push_back(BVerticesInA[FaceVertexNeighbors[bestFaceOnB*4+i]]);
      }
      *maxPatchClearance = getMaxPolygonClearance(face);
    }

    if (outPatch != 0) {
      //Enumerate all vertices on B around the selected face.
      //If a vertex is within a threshold of A's face's plane,
      //add it to the patch. Otherwise, if it has a neighbor that is 
      //within the threshold, compute the interpolated point where 
      //the edge passes the threshold and add that point to the
      //patch.
      double BFaceVertexDistances[4];
      Vector3 AFaceCorner = AVertices[FaceVertexNeighbors[bestFaceOnA*4]];
      //    Vector3 AFaceNormal = boxA.pose.rot * faceNormals[bestFaceOnA];
      Vector3 AFaceNormal = faceNormals[bestFaceOnA];
      int lowestAcceptedVertex = -1;
      for (int i = 0; i < 4; i++) {
	BFaceVertexDistances[i] = dot(AFaceNormal,
	    BVerticesInA[FaceVertexNeighbors[bestFaceOnB*4+i]] - AFaceCorner);
	if (BFaceVertexDistances[i] <= patchThreshold) {
	  if (lowestAcceptedVertex == -1) {
	    lowestAcceptedVertex = i;
	  }
	}
      }
      vector<Vector3> patchOnB;
      if (lowestAcceptedVertex != -1) {
	int i = lowestAcceptedVertex;
	int nexti = (i == 3 ? 0 : i+1);
	do  {
	  if (BFaceVertexDistances[i] <= patchThreshold) {
	    // Add vertex to patch
	    patchOnB.push_back(BVerticesInA[FaceVertexNeighbors[bestFaceOnB*4+i]]);

	    // If next vertex is too far away, find the point in between that crosses the threshold:
	    if (BFaceVertexDistances[nexti] > patchThreshold) {
	      double interpolationFactor = (patchThreshold - BFaceVertexDistances[i]) / (BFaceVertexDistances[nexti]-BFaceVertexDistances[i]);
	      Vector3 interpolatedPoint = 
		(1-interpolationFactor)*BVerticesInA[FaceVertexNeighbors[bestFaceOnB*4+i]] + 
		interpolationFactor * BVerticesInA[FaceVertexNeighbors[bestFaceOnB*4+nexti]];
	      patchOnB.push_back(interpolatedPoint);
	    }
	  }
	  else {
	    // If this vertex is too far away but the next one isn't, find the point in between that
	    // crosses the threshold:
	    if (BFaceVertexDistances[nexti] <= patchThreshold) {
	      double interpolationFactor = (patchThreshold - BFaceVertexDistances[i]) / (BFaceVertexDistances[nexti]-BFaceVertexDistances[i]);
	      Vector3 interpolatedPoint = 
		(1-interpolationFactor)*BVerticesInA[FaceVertexNeighbors[bestFaceOnB*4+i]] + 
		interpolationFactor * BVerticesInA[FaceVertexNeighbors[bestFaceOnB*4+nexti]];
	      patchOnB.push_back(interpolatedPoint);
	    }
	  }
	  i = nexti;
	  nexti = (i == 3 ? 0 : i+1);
	} 
	while (i != lowestAcceptedVertex);


	vector<Vector3>AFacePolygon;
	//Now, project the points onto the A face,
	//Need to reverse the winding on the bottom polygon for overlap computation
	vector<Vector3> patchOnA;
	for(int i = patchOnB.size()-1; i >= 0; i--) {
	  patchOnA.push_back(patchOnB[i] - AFaceNormal * dot(AFaceNormal, patchOnB[i]-AFaceCorner));
	}
	for(unsigned int i = 0; i < 4; i++) {
	  AFacePolygon.push_back(AVertices[FaceVertexNeighbors[bestFaceOnA*4+i]]);
	}

	//and find the overlap between it and the face. That's the contact patch!!1
	*outPatch = findPolygonIntersection(AFacePolygon, patchOnA);

	for (unsigned int i = 0; i < outPatch->size(); i++) {
	  (*outPatch)[i] = transform(boxA.pose, (*outPatch)[i]);
	}
      }
      else {
	// No point is close enough! Just return the distance
      }
    }
  }

  if (bestWitness.distance > 1e3)
    bestWitness.distance = 1e3;
  if (bestWitness.distance < -1e3)
    bestWitness.distance = -1e3;
  return bestWitness;
}

double
getMaxPolygonClearance(const std::vector<Vector3> &polygon) 
{
  //Find all bisectors
  std::vector<Vector3>bisectors;
  std::vector<Vector3>edgeNormals; // Pointing inward (left) from each edge [i+1]-[i]
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
  vector<double> minPositiveDistanceThisBisector(polygon.size(), FLT_MAX);

  for (unsigned int i = 0; i < polygon.size(); i++) {
    for (unsigned int j = i+1; j < polygon.size(); j++) {
      // Vector from i to j
      Vector3 difference = polygon[j] - polygon[i];

      // Component of above vector perpendicular to the
      // bisector at j
      Vector3 normalVector =
	difference - bisectors[j] * dot(difference, bisectors[j]);

      double normalLength = length(normalVector);

      // Inverse cosine of angle between bisector[i] and normalVector
      double invCosine = normalLength / (dot(normalVector, bisectors[i]));

      // Distance along bisector[i] to intersection with bisector[j]
      double intersectionParam =  normalLength * invCosine;

      double intersectionNormalDistance = 
	intersectionParam * dot(edgeNormals[i], bisectors[i]);
      if (intersectionNormalDistance > 0.0) {
	if (intersectionNormalDistance < minPositiveDistanceThisBisector[j]) {
	  minPositiveDistanceThisBisector[j] = intersectionNormalDistance;
	}
	if (intersectionNormalDistance < minPositiveDistanceThisBisector[i]) {
	  minPositiveDistanceThisBisector[i] = intersectionNormalDistance;
	}
      }
    }
  }
  for (unsigned int i = 0; i < polygon.size(); i++) {
    if (minPositiveDistanceThisBisector[i] < FLT_MAX &&
	minPositiveDistanceThisBisector[i] > maxDistance) {
      maxDistance = minPositiveDistanceThisBisector[i];
    }
  }
  return maxDistance;
}

};
