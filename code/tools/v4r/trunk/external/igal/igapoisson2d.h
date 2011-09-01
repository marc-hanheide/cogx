/*
 * iga.h
 *
 *  Created on: Oct 27, 2010
 *      Author: jbalzer
 */

#ifndef IGAPOISSON2D_H_
#define IGAPOISSON2D_H_

#include "ntools.h"
#include "SparseMat.h"

class Poisson2dIGA:public NURBSTools {

public:

	Poisson2dIGA(ON_NurbsSurface* surf);
	Poisson2dIGA(ON_NurbsSurface* surf, mat metricTensor);

	// main routine
	bool solve();

	// diffops
	void assembleStiffness();
	void assembleForceVector(cvec* cp);
	void assembleForceVector(trivariateScalarFunctionPointer func);
	void assembleForceVector(bivariateScalarFunctionPointer func);

	// bc
	bool setDirichlet(int I, int J, double val);
	bool setDirichletInterpolating(int I, int J, double val);
	bool unsetDirichlet(int I, int J) { return m_Dirichlet.erase(this->A(I,J)); };
	bool setNeumann(int I, int J, cvec dirVec, double val);
	bool unsetNeumann(int I, int J) { return m_Neumann.erase(this->A(I,J)); };;
	bool setHomogenousDirichlet(double offset = 0);
	bool setHomogenousNeumann(double offset = 0);
	void addNormalization(double normConst = 0);
	void removeNormalization();
	void resetLinearSystemSize();
	double verify(double u, double v, cvec* cpf);

	// access
	cvec getSolution() { return m_u; }
	SparseMat getStiffnessMatrix() { return m_K; };

protected:

	SparseMat m_K;

//	mat m_K;
	mat m_metricTensor;
	cvec m_u;
	cvec m_f;

	std::set<int> m_Dirichlet;					// keep track of Dirichlet vertices
	std::set<int> m_Neumann;					// keep track of Neumann vertices

};

#endif /* IGAPOISSON2D_H_ */
