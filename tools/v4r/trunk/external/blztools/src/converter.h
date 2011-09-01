#ifndef CONVERTER_H_
#define CONVERTER_H_

#include "blztools.h"

bool getScalarMeshFunction(cholmod_dense* funcVec, TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> propertyHandle); /*!< test */
bool getScalarMeshFunction(cholmod_sparse* funcMat, TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> propertyHandle);
bool getNormalMeshFunction(cholmod_dense* funcMat, TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> propertyHandle);
bool getPointMeshFunction(cholmod_dense* funcMat, TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Point> propertyHandle);
bool getMeshPoints(cholmod_dense* funcMat, TriangleMesh* M);
bool writeNormalMeshFunction(cholmod_dense* funcMat, TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Normal> propertyHandle, bool bc);
bool writeScalarMeshFunction(cholmod_dense* funcVec, TriangleMesh* M, OpenMesh::VPropHandleT<TriangleMesh::Scalar> propertyHandle);
bool writeNewVertices(cholmod_dense* funcMat, TriangleMesh* M, bool bc = false);

//LaVectorDouble convertVecToLapack(TriangleMesh::Normal vec);
//LaVectorDouble convertPtToLapack(TriangleMesh::Point vec);
//LaRowVectorDouble convertPtToLapackRow(TriangleMesh::Point vec);
//TriangleMesh::Point convertLapackToPt(LaVectorDouble vec);
//TriangleMesh::Normal convertLapackToVec(LaVectorDouble vec);
cvec convertVecToLapack(TriangleMesh::Normal vec);
cvec convertPtToLapack(TriangleMesh::Point vec);
rvec convertPtToLapackRow(TriangleMesh::Point vec);
TriangleMesh::Point convertLapackToPt(cvec vec);
TriangleMesh::Normal convertLapackToVec(cvec vec);

//Wm4::Vector3d convertVecToWm(TriangleMesh::Normal vec);
//Wm4::Vector3d convertPtToWm(TriangleMesh::Point vec);
//TriangleMesh::Point convertWmToPt(Wm4::Vector3d vec);
//TriangleMesh::Normal convertWmToVec(Wm4::Vector3d vec);

bool convertScannerToObj(char* fileNameIn, char* fileNameOut);

//bool convertLapackToCholmodSparse(LaGenMatDouble matrix, cholmod_sparse* sparseMatrix);
//LaGenMatDouble convertCholmodSparseToLapack(cholmod_sparse* sparseMatrix);
bool convertLapackToCholmodSparse(mat matrix, cholmod_sparse* sparseMatrix);
mat convertCholmodSparseToLapack(cholmod_sparse* sparseMatrix);


#endif /*CONVERTER_H_*/
