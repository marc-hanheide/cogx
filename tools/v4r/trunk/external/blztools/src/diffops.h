#ifndef DIFFOPS_H_
#define DIFFOPS_H_

#include "blztools.h"

bool solveSparseLinearSystem(cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x, bool transpose = false);
bool assembleLaplaceBeltramiOperator(cholmod_sparse* laplaceBeltrami, TriangleMesh* M, std::set<OpenMesh::VertexHandle>* boundaryHandles = NULL, bool weighted = false);
bool assembleLaplaceBeltramiOperatorNeumann(cholmod_sparse* laplaceBeltrami, TriangleMesh* M, std::set<OpenMesh::VertexHandle>* essentialBCHandles = NULL, bool weighted = false);
bool assembleLaplaceBeltramiOperatorNeumann(cholmod_sparse* laplaceBeltrami, cholmod_sparse* preCondJac, TriangleMesh* M, std::set<OpenMesh::VertexHandle>* essentialBCHandles = NULL, bool weighted = false);
bool assembleLaplacian(cholmod_sparse* laplaceBeltrami, TriangleMesh* M, bool weighted = false);
bool assembleLaplacianWithDirichletBc(cholmod_sparse* laplaceBeltrami, cholmod_sparse* boundaryMatrix, TriangleMesh* M, std::set<OpenMesh::VertexHandle>* boundaryHandles, bool weighted = false);
bool gaussianCurvature(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> curv);

bool principleCurvatures(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> maxCurv, OpenMesh::VPropHandleT<TriangleMesh::Scalar> minCurv);
bool principleCurvatureVectors(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> maxCurv, OpenMesh::FPropHandleT<TriangleMesh::Normal> minCurv);
bool meanCurvatureVector(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> curv, bool weighted = false);
bool absMeanCurvature(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> curv, bool weighted = false);
bool surfGradient(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> meshFunction, OpenMesh::FPropHandleT<TriangleMesh::Normal> surfGrad);
bool assembleEllipticOperator(cholmod_sparse* L, TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> c, std::set<OpenMesh::VertexHandle>* boundaryHandles = NULL, bool weighted = false);


//bool surfGradient(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> meshFunction, OpenMesh::VPropHandleT<TriangleMesh::Normal> surfGrad);
//bool surfJacobian(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> meshFunction, OpenMesh::VPropHandleT<LaGenMatDouble> surfJac);
//bool surfJacobian(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> meshFunction, OpenMesh::FPropHandleT<LaGenMatDouble> surfJac);
//LaVectorDouble surfGradientInVertex(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> meshFunction, OpenMesh::VertexHandle center);
//bool curvaturesInVertex(TriangleMesh* M, OpenMesh::VPropHandleT<LaGenMatDouble> ff, OpenMesh::VertexHandle vh, double* kappa, double* gamma, double* kmax, double* kmin, TriangleMesh::Normal* dirmax, TriangleMesh::Normal* dirmin);
//bool calcSecondFundamentalForm(TriangleMesh* M, OpenMesh::VPropHandleT<LaGenMatDouble> ff);
//bool calcSecondFundamentalForm(TriangleMesh* M, OpenMesh::FPropHandleT<LaGenMatDouble> ff);

bool surfJacobian(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> meshFunction, OpenMesh::VPropHandleT<mat> surfJac);
bool surfJacobian(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> meshFunction, OpenMesh::FPropHandleT<mat> surfJac);
TriangleMesh::Normal surfGradientInVertex(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> meshFunction, OpenMesh::VertexHandle center);
bool surfGradient(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> meshFunction, OpenMesh::VPropHandleT<TriangleMesh::Normal> surfGrad);
bool curvaturesInVertex(TriangleMesh* M, OpenMesh::VPropHandleT<mat> ff, OpenMesh::VertexHandle vh, double* kappa, double* gamma, double* kmax, double* kmin, TriangleMesh::Normal* dirmax, TriangleMesh::Normal* dirmin);
bool calcSecondFundamentalForm(TriangleMesh* M, OpenMesh::VPropHandleT<mat> ff);
bool calcSecondFundamentalForm(TriangleMesh* M, OpenMesh::FPropHandleT<mat> ff);



bool maxAbsPrincipleCurvature(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> mpc);
TriangleMesh::Normal meanCurvatureVector(TriangleMesh* M, OpenMesh::VertexHandle vh, bool weighted = false);
bool surfDivergence(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> vf, OpenMesh::VPropHandleT<TriangleMesh::Scalar> result, bool weighted = false);
bool surfRotation(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> vf, OpenMesh::VPropHandleT<TriangleMesh::Scalar> result, bool weighted = false);
bool distancePotential(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> result, OpenMesh::VertexHandle* center = NULL);
bool surfDivergence(QuadMesh* M, OpenMesh::FPropHandleT<QuadMesh::Normal> vf, OpenMesh::VPropHandleT<QuadMesh::Scalar> result);

//bool enforceNeumannBC(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> boundaryNormal, OpenMesh::FPropHandleT<mat> R);
bool enforceNeumannBC(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> boundaryNormal, OpenMesh::FPropHandleT<mat> R);



bool isInCornerFace(TriangleMesh* M, OpenMesh::VertexHandle vh, OpenMesh::VertexHandle* loc);
TriangleMesh::Normal meanCurvatureVectorInCorner(TriangleMesh* M, OpenMesh::VertexHandle vh);
TriangleMesh::Normal exteriorNormal(TriangleMesh* M, OpenMesh::VertexHandle vh);
TriangleMesh::Normal exteriorNormal(TriangleMesh* M, OpenMesh::FaceHandle fh);
double checkCompatibilityGap(TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> rhs);
bool conjugateDirection(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> dir, OpenMesh::FPropHandleT<TriangleMesh::Normal> conjDir);
bool solveSparseLinearSystemIteratively(cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x, double stepSize, int noOfSteps);
bool solveSparseLinearSystemLQ(cholmod_sparse* A, cholmod_dense* b, cholmod_dense* x);
bool scalarPotential(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> v, OpenMesh::VPropHandleT<TriangleMesh::Scalar> u);
bool vectorPotential(TriangleMesh* M, OpenMesh::FPropHandleT<TriangleMesh::Normal> v, OpenMesh::FPropHandleT<TriangleMesh::Normal> pot, OpenMesh::VPropHandleT<TriangleMesh::Scalar> w);

bool assembleGeneralLaplacian(cholmod_sparse* laplaceBeltrami, TriangleMesh* M, std::set<OpenMesh::VertexHandle> essentialBCHandles = std::set<OpenMesh::VertexHandle>(), std::vector<OpenMesh::HalfedgeHandle> naturalBCPath = std::vector<OpenMesh::HalfedgeHandle>(), bool weighted = false);

#endif /*DIFFOPS_H_*/
