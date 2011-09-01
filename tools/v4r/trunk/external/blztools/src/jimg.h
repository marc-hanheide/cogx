/*
 * jimg.hh
 *
 *  Created on: Mar 11, 2010
 *      Author: jbalzer
 */
#include "blztools.h"

#ifndef JIMG_HH_
#define JIMG_HH_


class JImg {

public:

	JImg(int sizeX = MAX_SIZE_X, int sizeY = MAX_SIZE_Y, int sizeZ = MAX_SIZE_Z, int sizeRange = 1, double hx = 1.0, double hy = 1.0, double hz = 1.0);
	JImg(JImg* img, int sizeRange);
	JImg(JImg* img, int iStart, int iEnd, int jStart, int jEnd, int kStart, int kEnd);
	~JImg();
	JImg(const JImg & img);
	JImg& operator= (const JImg & rhs);


	// access methods
	bool set(int i, int j, int k, cvec val);
	cvec get(int i, int j, int k);
	cvec get(int i) { return data_[i]; };
	cvec get(cvec x);
	bool put(cholmod_dense* f);
	bool get(cholmod_dense* f);
	double x(int i) { return x_[i]; };
	double y(int j) { return y_[j]; };
	double z(int k) { return z_[k]; };
	cvec x(int i, int j, int k);
	int shape(int dim) { return shape_[dim]; };
	double h(int dim) { return h_[dim]; };
	int i(double x);
	int j(double y);
	int k(double z);
	int i(int index);
	int j(int index);
	int k(int index);
	int assertIndex(int index, int dim);
	bool assertIndices(int i, int j, int k);
	bool isBoundary(int i, int j, int k);
	bool isNonZero(cvec x) { return (arma::norm(get(x),2)>0); };

	// extras
	cvec forwardDifference(int i, int j, int k, int dim);
	cvec backwardDifference(int i, int j, int k, int dim);
	//mat grad(int i, int j, int k);
	bool disturb(double noiseLevel = 0.1);
	bool normalize();
	double div(int i, int j, int k);
	cvec localMean(int ic, int jc, int kc, int h = 1);
	cvec localVar(int ic, int jc, int kc, int h = 1);
	mat jac(int i, int j, int k);
	bool laplacianDirichlet(cholmod_sparse* lb);
	bool laplacianNeumann(cholmod_sparse* lb);
	cvec outerNormal(int i, int j, int k);
	double max(int* iLoc, int* jLoc, int* kLoc);



    // i/o
	void print();
	bool printGrid();
	void print(int i, int j, int k);
	bool saveVTK(const char* fileName, char const* propName);
	bool loadVTK(const char* fileName);
	bool loadVTKStructuredPoints(const char* fileName);

	//bool saveVTKRadial(const char* fileName, char const* propName, double f);

	// grid
	bool setGrid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
	bool setGrid(cvec org, double hx = 1.0, double hy = 1.0, double hz = 1.0);
	bool assertGridsize();




protected:



	double* x_; 		/*!< x-values of grid */
	double* y_; 		/*!< y-values of grid */
	double* z_; 		/*!< z-values of grid */
	cvec* data_; 		/*!< actual data vector, length = MAX_SIZE_X*MAX_SIZE_Y*MAX_SIZE_Z */


	int shape_[4];		/*!< no of gridpoints, dimension of image */
	double h_[3];		/*!< grid sizes */

	static const unsigned int MAX_SIZE_X = 800;
	static const unsigned int MAX_SIZE_Y = 800;
	static const unsigned int MAX_SIZE_Z = 800;

};

#endif /* JIMG_HH_ */
