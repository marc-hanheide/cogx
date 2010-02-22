#ifndef RelationEvaluation_hpp
#define RelationEvaluation_hpp

#include <SpatialData.hpp>
//#include <Math.hpp>

using namespace SpatialData;
using namespace cogx::Math;

namespace spatial {

extern double squareDistanceWeight;
extern double squareDistanceFalloff;

extern double supportCOMContainmentOffset;
extern double supportCOMContainmentWeight;
extern double supportCOMContainmentSteepness;

extern double bottomCOMContainmentOffset;
extern double bottomCOMContainmentWeight;
extern double bottomCOMContainmentSteepness;

extern double planeInclinationWeight;

extern double overlapWeight;
// At this distance from COM to support edge,
// penalty is precisely 0.5


enum SpatialObjectType {OBJECT_PLANE, OBJECT_BOX, OBJECT_CYLINDER, OBJECT_SPHERE};

struct Object {
  SpatialObjectType type;

  cogx::Math::Pose3 pose;
};

enum PlaneObjectShape {PLANE_OBJECT_RECTANGLE, PLANE_OBJECT_CIRCLE};

struct PlaneObject : public Object {
  PlaneObjectShape shape;

  double radius1;
  double radius2;
};

struct BoxObject : public Object {
  double radius1;
  double radius2;
  double radius3;
};


double
evaluateOnness(const Object *objectS, const Object *objectO);

void
sampleOnnessDistribution(const Object *objectS, Object *objectO, 
    std::vector<Vector3> &outPoints) {

std::vector<Vector3>
findPolygonIntersection(const std::vector<Vector3> &polygon1, 
    const std::vector<Vector3> &polygon2);

double
findOverlappingArea(const std::vector<Vector3>& polygon, Vector3 circleCenter, double circleRadius, const Vector3 &circleNormal);

double
getPolygonArea(const std::vector<Vector3> &polygon);

double
getMaxPolygonClearance(const std::vector<Vector3> &polygon);
};
#endif //RelationEvaluation_hpp
