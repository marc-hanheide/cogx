#ifndef GVF_H_
#define GVF_H_

#include "blztools.h"

/*!
 * \brief Vector field on a regular Cartesian grid.
 *
 * Long description...
 */
class GridVectorField {

public:

	GridVectorField();
	GridVectorField(GridFunction* vx, GridFunction* vy, GridFunction* vz);
	virtual ~GridVectorField();

	GridFunction* vx_; 		/*!< pointer to x-component */
	GridFunction* vy_; 		/*!< pointer to y-component */
	GridFunction* vz_; 		/*!< pointer to z-component */
	//GridFunction* div_; 	/*!< pointer to divergence */

	// save
	bool save(const char* fileName, const char* propName);
	bool saveVTK(const char* fileName, char const* propName);
	bool loadVTK(const char* fileName);

	// vx,vy,vz public: access over functions belonging to gf class
	cvec get(int i, int j, int k);
	cvec x(int i, int j, int k);
	bool set(int i, int j, int k, cvec val);
	bool setGrid(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
	double div(int i, int j, int k);
	mat jac(int i, int j, int k);
	void print(int i, int j, int k) { std::cout << get(i,j,k) << std::endl; };

	bool normalize();
	bool disturb(double noiseLevel = 0.1);

private:

};

#endif /*GVF_H_*/
