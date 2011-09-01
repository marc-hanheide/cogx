#ifndef INTERP_H_
#define INTERP_H_

#include "blztools.h"

// TriangleMesh::Point interpFunctionLinear(TriangleMesh* M, OpenMesh::FaceHandle fh, TriangleMesh::Point point, OpenMesh::VPropHandleT<TriangleMesh::Point> func);
double interpFunctionLinear(TriangleMesh* M, OpenMesh::FaceHandle fh, TriangleMesh::Point point, OpenMesh::VPropHandleT<TriangleMesh::Scalar> func);
TriangleMesh::Normal interpFunctionLinear(TriangleMesh* M, OpenMesh::FaceHandle fh, TriangleMesh::Point point, OpenMesh::VPropHandleT<TriangleMesh::Normal> func);
TriangleMesh::Normal averageFunctionLinear(TriangleMesh* M, OpenMesh::VertexHandle vh, OpenMesh::FPropHandleT<TriangleMesh::Normal> func);
bool decimateMesh(TriangleMesh* M, unsigned int noOfVertices);
bool markSubsampledPoints(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> allowed, unsigned int noOfVertices);
bool nValidNeighborsBFS(TriangleMesh* M, OpenMesh::VertexHandle vh, OpenMesh::VPropHandleT<TriangleMesh::Scalar> allowed, unsigned int noOfNeighbors, std::set<OpenMesh::VertexHandle>* neighbors);
TriangleMesh decimateMeshIndexed(TriangleMesh* M, std::map<OpenMesh::VertexHandle, OpenMesh::VertexHandle>* highToLow, std::map<OpenMesh::VertexHandle,OpenMesh::VertexHandle>* lowToHigh, unsigned int noOfVertices);

#endif /*INTERP_H_*/
