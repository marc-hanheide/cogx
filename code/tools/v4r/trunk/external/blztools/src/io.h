#ifndef IO_H_
#define IO_H_

#include "blztools.h"


// triangle meshes
bool writeMeshToMshFormat(TriangleMesh* M, const char* fileName);
bool writeDataToMshFormat(TriangleMesh* M, const char* fileName, OpenMesh::VPropHandleT<TriangleMesh::Scalar> vertResHandle, const char* propName, int itIndex);
bool writeDataToMshFormat(TriangleMesh* M, const char* fileName, OpenMesh::FPropHandleT<TriangleMesh::Scalar> faceResHandle, const char* propName, int itIndex);
bool writeVecDataToMshFormat(TriangleMesh* M, const char* fileName, OpenMesh::VPropHandleT<TriangleMesh::Normal> vertResHandle, const char* propName, int itIndex);
bool writeVecDataToMshFormat(TriangleMesh* M, const char* fileName, OpenMesh::FPropHandleT<TriangleMesh::Normal> faceResHandle, const char* propName, int itIndex);
bool writeMeshToVtkFormat(TriangleMesh* M, const char* fileName, int itIndex = 0);
bool writeDataToVtkFormat(TriangleMesh* M, const char* fileName, OpenMesh::FPropHandleT<TriangleMesh::Scalar> faceResHandle, const char* propName, int itIndex = 0);
bool writeDataToVtkFormat(TriangleMesh* M, const char* fileName, OpenMesh::VPropHandleT<TriangleMesh::Scalar> vertResHandle, const char* propName, int itIndex = 0);
bool writeVecDataToVtkFormat(TriangleMesh* M, const char* fileName, OpenMesh::VPropHandleT<TriangleMesh::Normal> vertResHandle, const char* propName, int itIndex = 0);
bool writeVecDataToVtkFormat(TriangleMesh* M, const char* fileName, OpenMesh::FPropHandleT<TriangleMesh::Normal> faceResHandle, const char* propName, int itIndex = 0);
TriangleMesh createMeshToImage(TriangleMesh::Point origin, double edgeLengthU, double edgeLengthV, int width, int height, bool orientation = false);
bool writeSoup(TriangleMesh* M, const char* fileName, OpenMesh::FPropHandleT<mat> rotation, OpenMesh::FPropHandleT<TriangleMesh::Normal> translation = OpenMesh::FPropHandleT<TriangleMesh::Normal>(-1));


// quad meshes
bool writeQuadMeshToVtkFormat(QuadMesh* M, const char* fileName);
bool writeDataToVtkFormat(QuadMesh* M, const char* fileName, OpenMesh::VPropHandleT<QuadMesh::Scalar> vertResHandle, const char* propName);
bool writeDataToVtkFormat(QuadMesh* M, const char* fileName, OpenMesh::FPropHandleT<QuadMesh::Scalar> faceResHandle, const char* propName);
bool writeVecDataToVtkFormat(QuadMesh* M, const char* fileName, OpenMesh::VPropHandleT<QuadMesh::Normal> vertResHandle, const char* propName);
QuadMesh readQuadMeshFromVtkFormat(const char* fileName);
bool writeQuadMeshToMshFormat(QuadMesh* M, const char* fileName);
bool writeQuadDataToMshFormat(QuadMesh* M, const char* fileName, OpenMesh::VPropHandleT<QuadMesh::Scalar> vertResHandle, const char* propName, int itIndex);
bool writeQuadDataToMshFormat(QuadMesh* M, const char* fileName, OpenMesh::FPropHandleT<QuadMesh::Scalar> faceResHandle, const char* propName, int itIndex);
bool writeQuadVecDataToMshFormat(QuadMesh* M, const char* fileName, OpenMesh::VPropHandleT<QuadMesh::Normal> vertResHandle, const char* propName, int itIndex);
bool writeQuadVecDataToMshFormat(QuadMesh* M, const char* fileName, OpenMesh::FPropHandleT<QuadMesh::Normal> faceResHandle, const char* propName, int itIndex);
QuadMesh createQuadMeshToImage(QuadMesh::Point origin, double edgeLengthU, double edgeLengthV, int width, int height);
bool writeSoup(QuadMesh* M, const char* fileName, OpenMesh::FPropHandleT<mat> rotation, OpenMesh::FPropHandleT<QuadMesh::Normal> translation = OpenMesh::FPropHandleT<QuadMesh::Normal>(-1));
QuadMesh getSoup(QuadMesh* M, OpenMesh::FPropHandleT<mat> rotation, OpenMesh::FPropHandleT<QuadMesh::Normal> translation = OpenMesh::FPropHandleT<QuadMesh::Normal>(-1));

// other
bool writeOrientedPointsToVtkFormat(std::map<TriangleMesh::Point, TriangleMesh::Normal>* cloud, const char* fileName);
bool writeMeshToPovrayFormat(TriangleMesh* M, const char* fileName, double spec = 0.5, double diffuse = 0.5, double refl = 0.5);
bool writeLapackToMatFormat(mat matrix, const char* fileName);
bool readFixedVertexListFromFile(const char* fileName, std::set<OpenMesh::VertexHandle>* list);
bool writeMatrixtoMMFormat(mat A, const char* fileName);
bool writeHistogramm(double* bins, int* histogramm, int length, const char* fileName);


#endif /*IO_H_*/

