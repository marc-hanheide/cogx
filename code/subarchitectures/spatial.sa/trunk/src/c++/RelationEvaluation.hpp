#ifndef RelationEvaluation_hpp
#define RelationEvaluation_hpp

#include <SpatialData.hpp>
#include <vector>
//#include <Math.hpp>

using namespace SpatialData;
using namespace cogx::Math;

namespace spatial {

extern double patchThreshold;
extern double distanceFalloffOutside;
extern double distanceFalloffInside;
extern double supportCOMContainmentOffset;
extern double supportCOMContainmentSteepness;

// Old parameters
extern double squareDistanceWeight;
extern double squareDistanceFalloff;

extern double supportCOMContainmentWeight;

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

enum WitnessType {WITNESS_VERTEX, WITNESS_EDGE, WITNESS_FACE};
struct Witness {
  Vector3 point1;
  Vector3 point2;
  double distance;
  WitnessType typeOnA;
  WitnessType typeOnB;
  int idOnA; //Vertex, edge or face number 
  int idOnB; //Vertex, edge or face number 
  double paramOnA; //Parameter along edge
  double paramOnB;
  Vector3 normal; //Normal direction from A to B
};

double
evaluateOnness(const Object *objectS, const Object *objectO);

void
sampleOnnessDistribution(const Object *objectS, Object *objectO, 
    std::vector<Vector3> &outPoints, 
    double xmin, double xmax, 
    double ymin, double ymax,
    double zmin, double zmax, 
    double startStep, double minStep); 

std::vector<Vector3>
findPolygonIntersection(const std::vector<Vector3> &polygon1, 
    const std::vector<Vector3> &polygon2);

double
findOverlappingArea(const std::vector<Vector3>& polygon, Vector3 circleCenter, double circleRadius, const Vector3 &circleNormal);

double
getPolygonArea(const std::vector<Vector3> &polygon);

double
getDistanceToPolygon(const Vector3 &ref, const std::vector<Vector3> &polygon);

double
getMaxPolygonClearance(const std::vector<Vector3> &polygon);

Witness
findContactPatch(const BoxObject &boxA, const BoxObject &boxB, 
    std::vector<Vector3> *outPatch = 0);

bool
isIntersecting(double wr, double dr, double hr, const Vector3[]);

void
getCornerWitnesses(double wr, double dr, double hr, const Vector3 BVertices[],
    const std::vector<Vector3> & BEdges, std::vector<Witness> &cornerWitnesses);

void
getEdgeWitnesses(double wr, double dr, double hr, const Vector3 BVertices[],
    const std::vector<Vector3> & BEdges, std::vector<Witness> &edgeWitnesses,
    bool intersecting);

void
randomizeOrientation(Pose3 &pose);
};
#endif //RelationEvaluation_hpp
