/*
 * PatchFitting.h
 *
 *  Created on: Apr 5, 2011
 *      Author: moerwald
 */

#ifndef PATCHFITTING_H_
#define PATCHFITTING_H_

#include "ntools.h"
#include "SparseMat.h"
#include "DataLoading.h"

class PatchFitting{
public:
	ON_TextLog m_out;
	ON_NurbsSurface* m_patch;

	PatchFitting(int order, DataSet *data);
	~PatchFitting();

	inline void useArmadillo(){ use_arma = true; }
	inline void useCholmod(){ use_arma = false; }

	void refine(int dim);
//	void assemble_minimal_surface(int resU=64, int resV=64);
	void assemble(	int resU, int resV,
					double wBnd, double wInt,
					double wCurBnd, double wCurInt,
					double wCageRegBnd, double wCageReg,
					double wCorner);

	void solve(double damp=1.0);

	void updateSurf(double damp);

	void getQuadrangulation(int res0, int res1, QuadMesh &mesh);
	void getCage(QuadMesh &mesh);


protected:

	enum PatchSide{
		NORTH,
		WEST,
		SOUTH,
		EAST
	};
	class myvec{
	public:
		int side;
		double hint;
		myvec(int side, double hint){ this->side=side; this->hint=hint; }
	};

	void solve_arma(double damp);
	void solve_cholmod(double damp);

	void addPointConstraint(cvec params, cvec point, double weight, int& row);
	void addBoundaryPointConstraint(double paramU, double paramV, double weight, int &row);

	// TODO
	void addNormalConstraint(double weight, int& row);
	void addNormalStiffness(std::map< int, std::map< int, std::vector<cvec> > > &grouping, SparseMat &K);
	void addNormalForce(std::map< int, std::map< int, std::vector<cvec> > > &grouping, mat &f);

	void addCageRegularisation(double weight, int &row);
	void addCageBoundaryRegularisation(double weight, PatchSide side, int &row);
	void addCageCornerRegularisation(double weight, int &row);

	void addInteriorRegularisation(int order, int resU, int resV, double weight, int &row);
	void addBoundaryRegularisation(int order, int resU, int resV, double weight, int &row);

	cvec inverseMappingBoundary(cvec pt, int maxSteps=100, double accuracy=1e-5);
	cvec inverseMappingBoundary(cvec pt, int side, double hint, double &error, int maxSteps=100, double accuracy=1e-5, bool quiet=true);
	cvec inverseMapping(cvec pt, cvec* phint, int maxSteps, double accuracy, bool quiet);

	bool use_arma;
	bool use_int_hints;

	mat m_Karma;
	mat m_f;
	mat m_x;

	SparseMat m_Ksparse;

	std::vector<cvec> m_int_hints;

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
//	ON_PointCloud m_boundary;
//	ON_PointCloud m_interior;

};

#endif /* PATCHFITTING_H_ */
