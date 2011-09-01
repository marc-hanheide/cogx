#ifndef FEATURE_H_
#define FEATURE_H_

#include "blztools.h"

bool geodesicDistSubsampled(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> sig, OpenMesh::VPropHandleT<TriangleMesh::Scalar> allowed, int stepSize, double toleranceFactor);
bool findGeodesicAntipodes(TriangleMesh* M, OpenMesh::VertexHandle seed, std::set<OpenMesh::VertexHandle>* result, int noOfSteps = 5);
bool findGeodesicAntipodesIter(TriangleMesh* M, std::set<OpenMesh::VertexHandle>* result, OpenMesh::VertexHandle seed, int noOfStepsInner = 5, int noOfStepsOuter = 5);
OpenMesh::VertexHandle findKarcherMean(TriangleMesh* M);
//OpenMesh::VertexHandle findApproximateKarcherMean(TriangleMesh* M);
OpenMesh::VertexHandle findApproximateKarcherMean(TriangleMesh* M);
bool findExtremities(TriangleMesh* M, std::set<OpenMesh::VertexHandle>* result);

#endif /*FEATURE_H_*/
