#ifndef PROJECTIONOPS_H_
#define PROJECTIONOPS_H_

#include "blztools.h"

ANNkd_tree* buildAnnTree(TriangleMesh* M, std::vector<OpenMesh::VertexHandle>* pointToVertexHandle);
TriangleMesh::VertexHandle projectPointOnMeshAnn(TriangleMesh::Point preImage, ANNkd_tree* kdTree, std::vector<OpenMesh::VertexHandle>* pointToVertexHandle);
bool projectMeshOnMeshAnn(TriangleMesh* sourceMesh, TriangleMesh* targetMesh, TriangleMesh* projMesh, ANNkd_tree* searchTree, std::vector<OpenMesh::VertexHandle>*  pointToVertexHandle);
bool projectMeshOnMeshAnn(TriangleMesh* sourceMesh, TriangleMesh* targetMesh, TriangleMesh* projMesh, ANNkd_tree* searchTreeInterior, ANNkd_tree* searchTreeBoundary, std::vector<OpenMesh::VertexHandle>*  pointToVertexHandleInterior, std::vector<OpenMesh::VertexHandle>*  pointToVertexHandleBoundary);
TriangleMesh::Point projectPointOnOneRing(TriangleMesh::Point preImage, TriangleMesh* M, OpenMesh::VertexHandle center, OpenMesh::FaceHandle* fh = NULL);
TriangleMesh::Scalar distOfPointToMeshAnn(TriangleMesh::Point preImage, TriangleMesh* targetMesh, ANNkd_tree* searchTree, std::vector<OpenMesh::VertexHandle>*  pointToVertexHandle);
std::vector<TriangleMesh::Point> intersectLineWithMesh(TriangleMesh::Point basePoint, TriangleMesh::Normal dirVec, TriangleMesh* targetMesh, ANNkd_tree* searchTree, std::vector<OpenMesh::VertexHandle>* pointToVertexHandle);
TriangleMesh::Point intersectLineWithMeshCP(TriangleMesh::Point basePoint, TriangleMesh::Normal dirVec, TriangleMesh* targetMesh, ANNkd_tree* searchTree, std::vector<OpenMesh::VertexHandle>* pointToVertexHandle, OpenMesh::FaceHandle* fh = NULL);
OpenMesh::FaceHandle getFaceHandleOfProjectedPoint(TriangleMesh::Point preImage, TriangleMesh::Point* imagePoint, TriangleMesh* targetMesh, ANNkd_tree* searchTree, std::vector<OpenMesh::VertexHandle>*  pointToVertexHandle);
bool projectVectorFieldOnMesh(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> meshFunction);
bool projectVectorFieldOnMesh(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> meshFunction);
ANNkd_tree* buildAnnTree(TriangleMesh* M, OpenMesh::VPropHandleT<bool> constraint, std::vector<OpenMesh::VertexHandle>* pointToVertexHandle);
TriangleMesh::Point projectOnTriangle(TriangleMesh::Point p, TriangleMesh* M, OpenMesh::FaceHandle fh, double* dist = NULL);
bool distanceFunction(TriangleMesh* M, GridFunction* dist);
bool distanceFunction(TriangleMesh* M, JImg* dist, bool sign = false);


#endif /*PROJECTIONOPS_H_*/
