#ifndef UTILS_H_
#define UTILS_H_

#include "blztools.h"

// basic geometric operations
double cotanWeight(TriangleMesh::HalfedgeHandle hh, TriangleMesh* M);
double cotanOppositeAngle(TriangleMesh::HalfedgeHandle hh, TriangleMesh* M);
TriangleMesh::Point calcCenterOfGravity(TriangleMesh* M, OpenMesh::FaceHandle fh);
QuadMesh::Point calcCenterOfGravity(QuadMesh* M, OpenMesh::FaceHandle fh);
bool oneRingAngles(TriangleMesh* M, OpenMesh::VertexHandle center, std::vector<double>* angles);
double calcVoronoiArea(TriangleMesh* M, OpenMesh::VertexHandle center);
TriangleMesh::Point barycentricCoordinates(TriangleMesh* M, OpenMesh::FaceHandle fh, TriangleMesh::Point point);
double maxEdgeLength(TriangleMesh* M);
double minEdgeLength(TriangleMesh* M);
bool isTriangleObtuse(TriangleMesh* M, OpenMesh::FaceHandle fh);
bool isTriangleDegenerate(TriangleMesh* M, OpenMesh::FaceHandle fh, double tolerance = 0.01);
double totalArea(TriangleMesh* M);
double oneRingArea(TriangleMesh* M, OpenMesh::VertexHandle vh);
double faceArea(TriangleMesh* M, OpenMesh::FaceHandle fh);
TriangleMesh::Point calcCenterOfGravity(TriangleMesh* M);
bool calcBoundaryTangent(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> tangent);
bool calcBoundaryTangent(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> tangent);
TriangleMesh::Normal dualEdgeVector(TriangleMesh* M, OpenMesh::HalfedgeHandle heh);
double diagonalDistance(QuadMesh* M, OpenMesh::FaceHandle fh, OpenMesh::VertexHandle* vMin = NULL, OpenMesh::VertexHandle* vOppositeMin = NULL);
//LaVectorDouble boundingBox(TriangleMesh* M, double tolerance);
cvec boundingBox(TriangleMesh* M, double tolerance = 0);

// mesh processing
bool simpleMeshSmooth(TriangleMesh* M, int noOfSteps, bool fixBoundaries = false);
bool disturbVertices(TriangleMesh* M, double noiseLevel = 0.1);
bool translateMesh(TriangleMesh* M, TriangleMesh::Normal t);
bool blendBoundaryFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> blendOrder, int levels);
//bool rotateMeshAroundCOG(TriangleMesh* M, LaGenMatDouble R);
bool rotateMeshAroundCOG(TriangleMesh* M, mat R);
//bool integrateSoup(TriangleMesh* M, OpenMesh::FPropHandleT<LaGenMatDouble> R, std::set<OpenMesh::VertexHandle>* fixedVerts);
bool integrateSoup(TriangleMesh* M, OpenMesh::FPropHandleT<mat> R, std::set<OpenMesh::VertexHandle>* fixedVerts);
bool simpleBoundarySmooth(TriangleMesh* M, int noOfSteps);


// mesh topology
std::set<OpenMesh::VertexHandle> addGhostNodes(TriangleMesh* M);
bool collectBoundaryVertexHandles(TriangleMesh* M, std::set<OpenMesh::VertexHandle>* vertHandles);
bool deleteIsolatedVertices(TriangleMesh* M);
bool deleteIsolatedFaces(TriangleMesh* M);
bool deleteBoundary(TriangleMesh* M);
bool deleteVertices(TriangleMesh* M, std::set<OpenMesh::VertexHandle> vertices, bool boundary = false);
bool areNeighbors(TriangleMesh* M, OpenMesh::VertexHandle vh1, OpenMesh::VertexHandle vh2);
OpenMesh::VertexHandle findInteriorNeighbor(TriangleMesh* M, OpenMesh::VertexHandle vh);
OpenMesh::VertexHandle findBoundaryVertex(TriangleMesh* M);
OpenMesh::VertexHandle findClosestBoundaryVertex(TriangleMesh* M, OpenMesh::VertexHandle vh);
OpenMesh::HalfedgeHandle nextBoundaryHalfedge(TriangleMesh* M, OpenMesh::VertexHandle vh);
OpenMesh::HalfedgeHandle prevBoundaryHalfedge(TriangleMesh* M, OpenMesh::VertexHandle vh);
double edgeDistancesDijkstra(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> dist, OpenMesh::VertexHandle seed);
bool approximateCrestFaces(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Scalar> cf, int noiseLevel);
bool fastPatchBoundaries(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> edge, int noiseLevel);
int numberOfBoundaryVertices(TriangleMesh* M, OpenMesh::FaceHandle fh);
bool isBoundary(TriangleMesh* M, OpenMesh::FaceHandle fh);
bool containsVertex(TriangleMesh* M, OpenMesh::FaceHandle fh, OpenMesh::VertexHandle vh);
OpenMesh::HalfedgeHandle oppositeHalfedgeHandle(TriangleMesh* M, OpenMesh::FaceHandle fh, OpenMesh::VertexHandle vh);
OpenMesh::VertexHandle nextBoundaryVertex(TriangleMesh* M, OpenMesh::VertexHandle vh);
OpenMesh::VertexHandle previousBoundaryVertex(TriangleMesh* M, OpenMesh::VertexHandle vh);
bool vectorFeldIndices(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> v, OpenMesh::VPropHandleT<TriangleMesh::Scalar> index);
TriangleMesh subdivideSqrt3(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Scalar> criterion, double threshold);
TriangleMesh subdivideLoop(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Scalar> criterion, double threshold);
TriangleMesh subdivideUniformLoop(TriangleMesh* M, int noOfTimes);


bool collapseEdgesLength(TriangleMesh* M, double threshold);
bool collapseEdgesAspectRatio(TriangleMesh* M, double threshold = 0.1);
bool deleteSmallFaces(TriangleMesh* M, double threshold = 0.1);


// processing of functions
bool simpleMeshFunctionSmooth(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, int noOfSteps);
bool laplacianMeshFunctionSmooth(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, int noOfSteps, bool weighted = false);
bool laplacianMeshFunctionSmooth(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> function, int noOfSteps, bool weighted = false);
bool simpleMeshFunctionSmooth(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> function, int noOfSteps);
bool simpleMeshFunctionSmooth(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> function, int noOfSteps);
bool medianFilterMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function);
bool binarizeMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, double threshold);
bool erodeBinaryMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, int noOfSteps);
bool erodeBinaryMeshFunction(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Scalar> function, int noOfSteps);
bool dilateBinaryMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, int noOfSteps);
bool dilateBinaryMeshFunction(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Scalar> function, int noOfSteps);
bool closeBinaryMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, int noOfSteps);
bool closeBinaryMeshFunction(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Scalar> function, int noOfSteps);
bool normalizeMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, double boundFromOne);
double minMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, OpenMesh::VertexHandle* loc = NULL, bool onBoundary = false);
double maxMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, OpenMesh::VertexHandle* loc = NULL);
double maxMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, std::vector<OpenMesh::VertexHandle>* loc, int n = 1);
double meanMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function);
bool featureMorphology(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, OpenMesh::VPropHandleT<TriangleMesh::Normal> morph, double epsilon);
double binaryMeshSegmentation(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> feature, OpenMesh::VPropHandleT<TriangleMesh::Scalar> result, double scale);
double minLink(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, OpenMesh::VertexHandle vh, OpenMesh::VertexHandle* loc = NULL);
double maxLink(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, OpenMesh::VertexHandle vh, OpenMesh::VertexHandle* loc = NULL);
bool isConstantAroundVertex(TriangleMesh* M, OpenMesh::VertexHandle vh, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, double tol);
bool isLocallyMaximal(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, OpenMesh::VertexHandle vh, double tolerance);
bool localMaxima(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, std::set<OpenMesh::VertexHandle>* max, double tolerance);
TriangleMesh::Normal faceMean(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> function, OpenMesh::FaceHandle fh);
bool rectifyMeshFunction(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function);
bool localVariance(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, OpenMesh::VPropHandleT<TriangleMesh::Scalar> variance);
bool linearHistogramm(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, double* bins, int* histogramm, int length);
bool binaryHistrogrammSegmentation(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function, std::set<OpenMesh::VertexHandle>* foreGround, std::set<OpenMesh::VertexHandle>* backGround, double stoppingCriterion = 0.001);


// math utils
//bool minMaxEigenvectorDecomposition(LaGenMatDouble Ain, TriangleMesh::Normal* max, TriangleMesh::Normal* min, TriangleMesh::Normal kernel);
bool minMaxEigenvectorDecomposition(mat Ain, TriangleMesh::Normal kernel, TriangleMesh::Normal* max, TriangleMesh::Normal* min);


// experimental
TriangleMesh sliceMesh(TriangleMesh* M, OpenMesh::VertexHandle seed, std::vector<OpenMesh::VertexHandle>* leftBorder, std::vector<OpenMesh::VertexHandle>* rightBorder);
std::vector<OpenMesh::HalfedgeHandle> getBoundaryPath(TriangleMesh* M, OpenMesh::VertexHandle start = TriangleMesh::InvalidVertexHandle);
std::vector<OpenMesh::HalfedgeHandle> getSlicedBoundaryPath(TriangleMesh* M, OpenMesh::VertexHandle seed, std::set<OpenMesh::VertexHandle>* leftBorder, std::set<OpenMesh::VertexHandle>* rightBorder);
std::vector<OpenMesh::HalfedgeHandle> getSlicedBoundaryPath(TriangleMesh* M, OpenMesh::VertexHandle seed);
std::vector<OpenMesh::HalfedgeHandle> getSlicedBoundaryPath(TriangleMesh* M, OpenMesh::VertexHandle seed, OpenMesh::VPropHandleT<TriangleMesh::Scalar> function);
std::vector<OpenMesh::HalfedgeHandle> getSlicePath(TriangleMesh* M, OpenMesh::VertexHandle seed);

#endif /*UTILS_H_*/
