/*
 * dmL1.h
 *
 *  Created on: Mar 16, 2011
 *      Author: moerwald
 */

#ifndef DML1_H_
#define DML1_H_

#include "ntools.h"
#include "SparseMat.h"
#include <lpsolve/lp_lib.h>

class DepthMapL1:public NURBSTools{
public:
	DepthMapL1(ON_NurbsSurface* surf);
	//virtual ~DepthMap();

	void assembleStiffness();
	bool assembleForceVector(JImg2d *vf);

//	void resetLinearSystemSize();

	bool refine(int dim, int n = 1);
	void refine(int dim, double loc, int mult);

	void setDirichlet(int I, int J, double b=0.0);
	void setDirichlet();
	void setNeumann(int I, int J, JImg2d* vf);
	void setNeumann(JImg2d* vf);
	void setNormalization();

	void createLP();

	void solve_lp_solve();

	std::vector<double> & getSolution(){ return m_x; }

	void save(const char* path);

protected:

	int m_scale[2];

	int m_px_rows;
	int m_px_cols;

	std::vector<double> m_f0;
	std::vector<double> m_f1;
	SparseRowMat m_K0;
	SparseRowMat m_K1;

	SparseRowMat m_A;				// inequality constraint: A x <= b
	std::vector<double> m_b;	// inequality constraint: A x <= b
	SparseRowMat m_Ab;				// equality constraint: Ab x = bb
	std::vector<REAL>  m_bb;	// equality constraint: Ab x = bb
	SparseVector m_c;				// objective function: min(c x)
	std::vector<REAL> m_x;		// state vector / solution: x

	void printOutLPSolutionType(int ret);
	void updateSurf();

};

#endif /* DML1_H_ */
