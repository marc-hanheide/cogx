#ifndef __BLZTOOLS_H__
#define __BLZTOOLS_H__

#ifndef INFINITY
#define INFINITY 1000000000
#endif

#include <iostream>
#include <stdio.h>
#include <vector>
#include <sys/stat.h>
#include <map>
#include <queue>
#include <list>
#include <algorithm>

// mesh data structure
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/RulesT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/LoopT.hh>


// dense matrices
//#include "armadillo-1.0.0/include/armadillo"
#include <armadillo>

// sparse matrices
#include <suitesparse/cholmod.h>
#include <suitesparse/umfpack.h>

// other tools
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ANN/ANN.h>

// combinatorial optimization
#include "matrix.h"
#include "munkres.h"
#include "graph.h"

// typedefs
typedef Matrix<double> SimpleMat;
typedef OpenMesh::TriMesh_ArrayKernelT<OpenMesh::Subdivider::Adaptive::CompositeTraits>  TriangleMesh;
typedef OpenMesh::Subdivider::Adaptive::CompositeT<TriangleMesh> Subdivider;
typedef OpenMesh::Subdivider::Uniform::LoopT<TriangleMesh> SubdividerULoop;
typedef OpenMesh::PolyMesh_ArrayKernelT<>  QuadMesh;
typedef arma::mat mat;
typedef arma::colvec cvec;
typedef arma::rowvec rvec;
typedef double (*bivariateScalarFunctionPointer)(double,double);
typedef double (*trivariateScalarFunctionPointer)(double,double,double);


// additional data structures
#include "gf.h"					/*!< scalar field on Cartesian grid */
#include "gvf.h"
#include "geoimg.h"				/*!< mesh with image or other measurement attached */
#include "jimg.h"
#include "jimg2d.h"

// the lib
#include "converter.h"	 		/*!< data type conversion */
#include "diffops.h"			/*!< differential operators */
#include "io.h"					/*!< file in/output */
#include "interp.h"				/*!< interpolation */
#include "projectionops.h"		/*!< projection */
#include "utils.h"				/*!< utilities */
#include "feature.h"			/*!< (differential) geometrical features */
#include "int.h"				/*!< numerical integration */



#endif /*__BLZTOOLS_H__*/
