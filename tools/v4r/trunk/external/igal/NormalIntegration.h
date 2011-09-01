/*
 * dm.h
 *
 *  Created on: Jan 26, 2011
 *      Author: jbalzer
 */

#ifndef DM_H_
#define DM_H_

#include "igapoisson2d.h"
#include "SparseMat.h"

class NormalIntegration: public Poisson2dIGA {
public:
	NormalIntegration(ON_NurbsSurface* surf);
	//virtual ~DepthMap();
	void useArmadillo();
	inline void useCholmod(){ use_arma = false; }

	void assembleStiffness();
	bool assembleForceVector(JImg2d* vf);

	bool refine(int dim, int n = 1);
	void refine(int dim, double loc, int mult);

	void setNatural(JImg2d* vf);

	void solve();



	double saveNormalError(JImg2d* vf, const char* fileName, const char* propName, int format = 0);

protected:

	bool use_arma;

	SparseMat m_Ksparse;

	mat m_Karma;
	mat m_u;
	mat m_f;
	int m_scale[2];
	void updateSurf();

	void solve_cholmod();
	void solve_arma();


};

#endif /* DM_H_ */
