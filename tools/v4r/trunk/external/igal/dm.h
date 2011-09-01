/*
 * dm.h
 *
 *  Created on: Jan 26, 2011
 *      Author: jbalzer
 */

#ifndef DM_H_
#define DM_H_

#include "igapoisson2d.h"

class DepthMap: public Poisson2dIGA {
public:
	DepthMap(ON_NurbsSurface* surf);
	//virtual ~DepthMap();

	void assembleStiffness();
	bool assembleForceVector(JImg2d* vf);

	bool refine(int dim, int n = 1);
	void refine(int dim, double loc, int mult);

	void setNatural(JImg2d* vf);

	void solve();

protected:

	int m_scale[2];
	void updateSurf();



};

#endif /* DM_H_ */
