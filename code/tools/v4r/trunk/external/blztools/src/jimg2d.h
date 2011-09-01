/*
 * djimg.h
 *
 *  Created on: Mar 13, 2010
 *      Author: jbalzer
 */

#include "jimg.h"

#ifndef JIMG2D_H_
#define JIMG2D_H_

class JImg2d:public JImg {

public:

	JImg2d();
	JImg2d(int sizeX, int sizeY, int sizeRange, cvec focalLength = arma::zeros(2,1), cvec principalPoint = arma::zeros(2,1), mat R = arma::eye(3,3), cvec t = arma::zeros(3,1));

	bool inFOV(cvec x, bool local = false);
	cvec project(cvec x, bool local = false);
	cvec transformTo(cvec x) { return arma::trans(R_)*(x - t_);};
	cvec transformFrom(cvec x) { return R_*x + t_; };
	cvec get(cvec x);
	cvec get(int i, int j) { return JImg::get(i,j,0); };
	bool printParams();
	bool isBoundary(int i, int j);
	bool loadParams(const char* fileName, bool invert = false);
	cvec pointOnPrincipleAxis(double distance);
	mat jac(int i, int j);
	double div(int i, int j);
	cvec outerNormal(int i, int j);
	bool islocalMaximum(int i, int j);
	std::vector<int> search(double val, int component, double acc = 1e-04);
	cvec getFocalLength() { return f_; };
	bool assertGridsize();

	TriangleMesh constructMeshFromDomain(bool orientiation = 0);
	TriangleMesh constructStaggeredMeshFromDomain(bool orientiation = 0);


protected:

	cvec f_;
	cvec pp_;
	mat R_;
	cvec t_;
	cvec k_;
	double alpha_;

};

#endif /* JIMG2D_H_ */
