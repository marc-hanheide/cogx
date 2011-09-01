/*
 * PatchFitting.h
 *
 *  Created on: Jun 8, 2011
 *      Author: moerwald
 */

#ifndef PATCHFITTING_H_
#define PATCHFITTING_H_

#include <opennurbs.h>
#include "DataLoading.h"

class PatchFitting{
public:
	ON_TextLog m_out;
	ON_NurbsSurface* m_patch;

	PatchFitting(int order, DataSet *data);
	~PatchFitting();


	double m_min[3];
	double m_max[3];
	double m_mid[3];

	std::vector<double> m_elementsU;
	std::vector<double> m_elementsV;

	double m_minU;
	double m_minV;
	double m_maxU;
	double m_maxV;

	DataSet* data;


};

#endif /* PATCHFITTING_H_ */
