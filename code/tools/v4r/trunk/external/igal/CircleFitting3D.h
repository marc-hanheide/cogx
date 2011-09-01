/*
 * CircleFitting3D.h
 *
 *  Created on: Apr 2, 2011
 *      Author: moerwald
 */

#ifndef CIRCLEFITTING3D_H_
#define CIRCLEFITTING3D_H_

#include "blztools/src/blztools.h"
#include <opennurbs.h>

class CircleFitting3D{
public:
	ON_PointCloud m_boundary;
	ON_TextLog m_out;

	CircleFitting3D(int order, const ON_PointCloud &boundary);
	~CircleFitting3D();

	void refine();
	void assemble();
	void solve_arma();
	void updateCurve();

	void getCurve(ON_PointCloud &pc_curve, unsigned res);
	void getCV(ON_PointCloud &cv);

	inline ON_NurbsCurve* getCurve(){ return m_curve; }

//	void loadPointCloud(const char* pointcloud_file);
//	static void savePointCloutTxt(const ON_PointCloud &pc, const char* txt_file);
//	static void savePointCloudVtk(const ON_PointCloud &pc, const char* vtk_file);
//	static void generateRandomPointCloud(const char* pointcloud_file, unsigned num=1000);

protected:

	mat m_K;
	mat m_f;
	mat m_x;

	double m_min[3];
	double m_max[3];
	double m_mid[3];

	std::vector<double> m_hints;
	bool use_hints;

	ON_NurbsCurve* m_curve;


	void init(int order);

	double inverseMapping(cvec pt, double *hint = NULL, int maxSteps = 100, double accuracy = 1e-6, int initial_points=3);
};

#endif /* CIRCLEFITTING3D_H_ */


// ********************************************************************************************
// 		Usage
// ********************************************************************************************

//	CircleFitting3D curveFit(circle_order, on_bnd);
//
//	for(int i=0; i<circle_refinement; i++)
//		curveFit.refine();
//
//	for(int i=0; i<circle_iterations; i++){
//		curveFit.assemble();
//		curveFit.solve_arma();
//	}
//
//	curveFit.getCurve(on_curve, 512);
//	curveFit.getCV(on_curvecage);
//
//	DepthMapPC::savePointCloudVtk(on_curve, curve_file_vtk.c_str());
//	DepthMapPC::savePointCloudVtk(on_curvecage, curve_cage_vtk.c_str());
