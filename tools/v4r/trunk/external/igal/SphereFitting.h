/*
 * SphereFitting.h
 *
 *  Created on: Apr 2, 2011
 *      Author: moerwald
 */

#ifndef SPHEREFITTING_H_
#define SPHEREFITTING_H_

#include "blztools/src/blztools.h"
#include <opennurbs.h>
#include "ntools.h"

class SphereFitting{
public:
	ON_PointCloud m_pointcloud;
	ON_TextLog m_out;

	SphereFitting(int order, const ON_PointCloud &pointcloud);
	SphereFitting(ON_NurbsSurface* sphere, const ON_PointCloud &pointcloud);
	~SphereFitting();

	void refine(int dim);
	void assemble();
	void solve_arma();
	void updateCurve();

	void getCurve(ON_PointCloud &pc_curve, unsigned res);
	void getCV(ON_PointCloud &cv);

	void getQuadrangulation(int res0, int res1, QuadMesh &mesh);

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

	std::vector<double> m_elements0;
	std::vector<double> m_elements1;
	std::vector<double> m_deltas0;
	std::vector<double> m_deltas1;

	double m_xi_max;
	double m_xi_min;
	double m_eta_max;
	double m_eta_min;

	int m_ncp0;
	int m_ncp1;


	std::vector<cvec> m_hints;
	bool use_hints;

	ON_NurbsSurface* m_sphere;


	void init(int order);

	cvec inverseMapping(cvec pt, cvec *hint = NULL, int maxSteps = 100, double accuracy = 1e-6, bool quiet=true);
};

#endif /* SPHEREFITTING_H_ */

// ********************************************************************************************
// 		Usage
// ********************************************************************************************
//
//	SphereFitting sphereFit(sphere_order, on_pc);
//
//	sphereFit.refine(0);
//	sphereFit.refine(1);
//
//	sprintf(charbuffer, "%ssphere_iter%d.vtk", file_path.c_str(), 0);
//	QuadMesh result;
//	sphereFit.getQuadrangulation(64,64, result);
//	writeQuadMeshToVtkFormat(&result, charbuffer);
//
//	for(int i=0; i<sphere_iterations; i++){
//		sphereFit.assemble();
//		sphereFit.solve_arma();
//
//		sprintf(charbuffer, "%ssphere_iter%d.vtk", file_path.c_str(), i+1);
//		sphereFit.getQuadrangulation(64,64, result);
//		writeQuadMeshToVtkFormat(&result, charbuffer);
//	}
