/*
 * PatchFitting.cpp
 *
 *  Created on: Jun 8, 2011
 *      Author: moerwald
 */

#include "PatchFitting.h"

PatchFitting::PatchFitting(int order, DataSet *data){

	ON::Begin();

	this->data = data;

	data->boundary.GetBBox(m_min,m_max);

	m_mid[0] = m_min[0] + (m_max[0]-m_min[0])*0.5;
	m_mid[1] = m_min[1] + (m_max[1]-m_min[1])*0.5;
	m_mid[2] = m_min[2] + (m_max[2]-m_min[2])*0.5;

	m_patch = new ON_NurbsSurface(3, true, order, order, order, order);
	m_patch->MakeClampedUniformKnotVector(0, 1.0);
	m_patch->MakeClampedUniformKnotVector(1, 1.0);

	double dcu = (m_max[0]-m_min[0])/(order-1);
	double dcv = (m_max[1]-m_min[1])/(order-1);

	ON_3dPoint cv(0.0,0.0,0.0);
	for(int i=0; i<order; i++) {
		for(int j=0; j<order; j++) {
			cv.x = m_min[0] + dcu * i;
			cv.y = m_min[1] + dcv * j;
			if(j==0)
				cv.z = m_min[2];
			else
				cv.z = m_max[2];
			m_patch->SetCV(i,j,cv);
		}
	}
}

PatchFitting::~PatchFitting(){

	delete(m_patch);

	ON::End();
}

