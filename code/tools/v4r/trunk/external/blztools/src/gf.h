#ifndef GF_H_
#define GF_H_

#include "blztools.h"

/*!
 * \brief Scalar function on a regular Cartesian grid.
 *
 * Long description...
 */
class GridFunction {

public:

	GridFunction();
	GridFunction(GridFunction* gf);
	GridFunction(int sizeX, int sizeY, int sizeZ, double hx = 1.0, double hy = 1.0, double hz = 1.0);
	//GridFunction(char* file, double hx = 1.0, double hy = 1.0, double hz = 1.0);
	~GridFunction();


	// access methods
	bool set(int i, int j, int k, double val);
	bool set(double x, double y, double z, double val);
	double get(int i, int j, int k);
	double get(int i) { return data_[i]; };
	double getScaleMean(int dim);
	double x(int i) { return x_[i]; };
	double y(int j) { return y_[j]; };
	double z(int k) { return z_[k]; };
	int i(double x);
	int j(double y);
	int k(double z);
	double forwardDifference(int i, int j, int k, int dim);
	double backwardDifference(int i, int j, int k, int dim);
	cvec grad(int i, int j, int k);
	bool setAnalytic();



	int shape(int dim) { return shape_[dim]; };
	double h(int dim) { return h_[dim]; };


    // i/o
	bool loadGridFunctionFromFile(char *);
	void print();
	bool printGrid();
	void print(int i, int j, int k) { printf("%f\n", get(i,j,k)); };
	bool save(const char* fileName, char const* propName);
	bool saveVTK(const char* fileName, char const* propName);
	bool saveVTKRadial(const char* fileName, char const* propName, double f);

	// grid
	void setGrid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
	void setGrid(double minX, double minY, double minZ, double h);
	bool set(int dim, int i, double val);
	bool initShape(unsigned int sizeX, unsigned int sizeY, unsigned int sizeZ);
	bool updateH();

	// arithmetic
	double  interpTriLinear(double x, double y, double z);
	bool add(GridFunction* gf);

private:


	double* x_; /*!< x-values of grid */
	double* y_; /*!< y-values of grid */
	double* z_; /*!< z-values of grid */
	double* data_; /*!< actual data vector, length = MAX_SIZE_X*MAX_SIZE_Y*MAX_SIZE_Z */


	int shape_[3];	/*!< no of gridpoints */
	double h_[3];	/*!< grid sizes */

	static const unsigned int MAX_SIZE_X = 200;
	static const unsigned int MAX_SIZE_Y = 200;
	static const unsigned int MAX_SIZE_Z = 200;



//	double x_[MAX_SIZE_X]; /*!< x-values of grid */
//	double y_[MAX_SIZE_Y]; /*!< y-values of grid */
//	double z_[MAX_SIZE_Z]; /*!< z-values of grid */
//	double data_[MAX_SIZE_X*MAX_SIZE_Y*MAX_SIZE_Z]; /*!< actual data vector, length = MAX_SIZE_X*MAX_SIZE_Y*MAX_SIZE_Z */




};

#endif /*GF_H_*/
