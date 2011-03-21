#ifndef RelationEvaluation_hpp
#define RelationEvaluation_hpp

//#include <SpatialData.hpp>
#include <vector>
#include <Math.hpp>

//using namespace SpatialData;
using namespace cogx::Math;

namespace spatial {
  enum SpatialObjectType {OBJECT_PLANE, OBJECT_BOX, OBJECT_CYLINDER, OBJECT_SPHERE, OBJECT_HOLLOW_BOX};

enum SpatialRelationType {RELATION_ON, RELATION_IN, RELATION_COMPOSITE};

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

struct CylinderObject : public Object {
  double radius1;
  double radius2;
};

struct SphereObject : public Object {
  double radius1;
};

//Box with the top (Z+) side open
struct HollowBoxObject : public BoxObject {
  double thickness;
  int sideOpen;
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




class RelationEvaluator {
  public:
    RelationEvaluator();



    typedef std::pair<int, int> Edge;

    struct Polyhedron {
      std::vector<Vector3> vertices;
      std::vector<std::vector<Edge> > faces; //Edge list around face (positive direction)
    };

  public:

    double
      evaluateInness(const Object *objectC, const Object *objectO);

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
      computePolyhedronVolume(const Polyhedron &polyhedron);

    void
      computeConvexHull(const std::vector<Vector3>& points, 
	  const Vector3 &polygonNormal, std::vector<Vector3>& hull);

    double
      getPolygonArea(const std::vector<Vector3> &polygon);

    double
      getPolygonAreaAndCentroid(const std::vector<Vector3> &polygon, 
	  Vector3 &centroid);

    Witness
      findContactPatch(const BoxObject &boxA, const BoxObject &boxB, 
	  std::vector<Vector3> *outPatch = 0, double *maxPatchClearance = 0);

    Vector3
      computeAttentionVectorSumForPatch(const std::vector<Vector3> patch,
	  const Vector3 &focus, const Vector3 &trajector, double falloff);

    Vector3
      computeAttentionVectorSumForSolid(const Object *obj,
	  const Vector3 &focus, const Vector3 &trajector, double falloff);

  protected:


    void
      mergeAnyOverlappingVertices(Polyhedron &polyhedron, double epsilon);

    void
      clipPolyhedronToPlane(Polyhedron &polyhedron, const Vector3 &pointInPlane,
	  const Vector3 &planeNormal);

    double
      findOverlappingArea(const std::vector<Vector3>& polygon, Vector3 circleCenter, double circleRadius, const Vector3 &circleNormal);

    double
      getDistanceToPolygon(const Vector3 &ref, const std::vector<Vector3> &polygon);

    double
      getMaxPolygonClearance(const std::vector<Vector3> &polygon);

    bool
      isIntersecting(double wr, double dr, double hr, const Vector3[]);

    void
      getCornerWitnesses(double wr, double dr, double hr, const Vector3 BVertices[],
	  const std::vector<Vector3> & BEdges, std::vector<Witness> &cornerWitnesses);

    void
      getEdgeWitnesses(double wr, double dr, double hr, const Vector3 BVertices[],
	  const std::vector<Vector3> & BEdges, std::vector<Witness> &edgeWitnesses,
	  bool intersecting);


  public:
    //Onness parameters
    double patchThreshold;
    double distanceFalloffOutside;
    double distanceFalloffInside;
    double supportCOMContainmentOffset;
    double supportCOMContainmentSteepness;

    //Inness parameters
    double planeThickness;
    double circlePlaneApproximationThreshold; //Controls number of
    //edges in polygon used to approximate circular planes
    double cylinderApproximationThreshold;
    int sphereTessellationFactor; //Number of latitudes and half number of
    //longitudes. 2 makes the sphere an octahedron
    double boxThickness; //Controls thickness of walls of hollow container

    Witness m_lastWitness;
    double m_lastDistanceFactor;
    std::vector<Vector3> m_lastPatch;
};

void
randomizeOrientation(Pose3 &pose);

bool
inferRelationsThreeObjects(std::vector<double> &ret, double BOnA, double AOnB, double BOnT,
   double AOnT, double BInA, double AInB, double BInT, double AInT);

void
getRandomSampleSphere(std::vector<Matrix33> &orientations, int n);

void
getRandomSampleCircle(std::vector<Matrix33> &orientations, int n);

spatial::Object *
generateNewObjectModel(const std::string &label);

};
#endif //RelationEvaluation_hpp
