/*
 * ntools.h
 *
 *  Created on: Nov 22, 2010
 *      Author: jbalzer
 */

#ifndef NTOOLS_H_
#define NTOOLS_H_

#include <opennurbs.h>
#include "blztools/src/blztools.h"

class NURBSTools {

public:

	enum { NORTH = 1, NORTHEAST = 2, EAST = 3, SOUTHEAST = 4, SOUTH = 5, SOUTHWEST = 6, WEST = 7, NORTHWEST = 8 };

	NURBSTools(ON_NurbsSurface* surf);
	//virtual ~NURBSTools();


	// evaluations in the parameter domain
	double eval(double u, double v, cvec* cp);
	cvec eval(double u, double v, mat* cp);
	double evalDerivative(double u, double v, cvec* cp, int dim, int no = 1);
	double laplace(double u, double v, cvec* cp);
	double biLaplace(double u, double v, cvec* cp);
	mat surfJac(double u, double v, mat* cp);
	cvec grad(double u, double v, cvec* cp);
	cvec surfGrad(double u, double v, cvec* cp);
	mat jac(double u, double v, mat* cp);
	cvec x(double u, double v);
	cvec xv(double u, double v);
	cvec n(double u, double v);
	mat jacX(double u, double v);
	cvec normalComponent(double u, double v, cvec vec);
	cvec tangentialComponent(double u, double v, cvec vec);
	cvec conjugateDirection(double u, double v, cvec vec);

	JImg2d getFuncOverRegularParamGrid(cvec* cp, int resU, int resV);  // what about irregular param grids?
	double integrate(cvec* cp);
	double integrate(trivariateScalarFunctionPointer func);
	std::vector<double> getElementVector(int dim);
	std::vector<double> getElementVectorDeltas(int dim);
	std::vector<double> getUniformResampledElementVector(int dim, int noOfElements);
	cvec inverseMapping(cvec pt, cvec hint, int maxSteps = 100, double accuracy = 1e-6, bool quiet=false);
	double inverseMapping(cvec pt, double hint, int side, int maxSteps = 100, double accuracy = 1e-6);

	// utils
	cvec outerNormal(double u, double v);
	int dir(int I, int J);
	int dir(double u, double v, double accuracy = 1e-1);
	bool isBoundary(int I, int J) { return (I==0 || J==0 || I==(m_surf->m_cv_count[0]-1) || J==(m_surf->m_cv_count[1]-1)); }
	cvec approximateFunction(bivariateScalarFunctionPointer func,  int noOfElements);
	bool deform(mat* cp);

	bool save(const char* fileName, cvec* cp, const char* propName, std::vector<double> elementsU, std::vector<double> elementsV, int format = 0);
	bool save(const char* fileName, mat* cp, int component, const char* propName, std::vector<double> elementsU, std::vector<double> elementsV, int format = 0);
	bool saveNURBSCoords(const char* fileName, std::vector<double> elementsU, std::vector<double> elementsV, int dim, int format = 0);
	QuadMesh getCage();
	void getQuadrangulation(int resU, int resV, QuadMesh &mesh);
	QuadMesh getQuadrangulation(std::vector<double> elementsU, std::vector<double> elementsV);

	TriangleMesh getTriangulation(std::vector<double> elementsU, std::vector<double> elementsV);

	ON_NurbsSurface* m_surf;

	// index routines
	int A(int I, int J) { return m_surf->m_cv_count[1]*I + J; }																// two global indices to one global index (lexicographic)
	int a(int i, int j) { return m_surf->m_order[1]*i + j; }																// two local indices into one local index (lexicographic)
	int i(int a) { return (int)(a/m_surf->m_order[1]); }																	// local lexicographic in local row index
	int j(int a) { return (int)(a%m_surf->m_order[1]);}																		// local lexicographic in local col index
	int I(int A) { return (int)(A/m_surf->m_cv_count[1]); }																	// global lexicographic in global row index
	int J(int A) { return (int)(A%m_surf->m_cv_count[1]); }																	// global lexicographic in global col index
	int A(int E, int F, int i, int j) { return A(E+i, F+j); };																// check this: element + local indices to one global index (lexicographic)
	int E(double u) { return ON_NurbsSpanIndex(m_surf->m_order[0],m_surf->m_cv_count[0],m_surf->m_knot[0],u,0,0); }			// element index in u-direction
	int F(double v) { return ON_NurbsSpanIndex(m_surf->m_order[1],m_surf->m_cv_count[1],m_surf->m_knot[1],v,0,0); }			// element index in v-direction

	protected:

};

#endif /* NTOOLS_H_ */
