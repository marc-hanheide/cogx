/*
 * evol.h
 *
 *  Created on: 03.12.2008
 *      Author: jbalzer
 */

#ifndef __EVOL_H__
#define __EVOL_H__

#include "blztools.h"

class MeshEvolution {

public:

	MeshEvolution(char* sourceMeshName, double stepSize, int noOfSteps, double stopThreshold, bool normalMode);
	MeshEvolution(TriangleMesh M, double stepSize, int noOfSteps, double stopThreshold, bool normalMode);
	MeshEvolution(double stepSize, int noOfSteps, double stopThreshold, bool normalMode);
    //virtual ~MeshEvolution() {};

	bool iterate(int saveStep);

	bool saveCurrent();
	bool saveResidual(const char* fileName = "residual.dat");
	bool saveCurrent(char* fileName);


	TriangleMesh::Point getPoint(OpenMesh::VertexHandle vh) { return itMesh_.point(vh); };
	TriangleMesh getMesh() { return itMesh_; };

protected:

	double stepSize_;
	double t_;
	int    k_;
	int    noOfSteps_;
	//double residual_;
	double stopThreshold_;
	bool   normalMode_;
	std::vector<double> residual_;


	TriangleMesh itMesh_;
	OpenMesh::VPropHandleT<TriangleMesh::Normal> conVelocity_;
	OpenMesh::VPropHandleT<TriangleMesh::Scalar> normalVelocity_;
	OpenMesh::VPropHandleT<TriangleMesh::Scalar> residualPerVertex_;
	OpenMesh::FPropHandleT<TriangleMesh::Scalar> residualPerFace_;


	void printResidual() { std::printf("Residual: %f\n",residual_.back());};
	void printTime() { std::cout << "Time: " << t_ << std::endl; }
    void printItIndex() { std::cout << "Iteration: " << k_ << std::endl; }
    bool checkCFLCondition();

	virtual bool doStep();
	virtual bool updateNormalVelocity(){ return 0; };
	virtual bool updateConVelocity(){ return 0; };
	virtual bool updateResidual(){ return 0; }
	virtual bool adaptStepSize() {return 0; };

};

#endif
