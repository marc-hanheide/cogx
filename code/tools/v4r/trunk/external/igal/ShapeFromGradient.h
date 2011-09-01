/*
 * ShapeFromGradient.h
 *
 *  Created on: Mar 27, 2011
 *      Author: jbalzer
 */

#ifndef SHAPEFROMGRADIENT_H_
#define SHAPEFROMGRADIENT_H_




enum BoundaryCondition{
	NEUMANN,
	DIRICHLET,
	NONE,
};

enum NormType{
	L2,
	L1,
};

class ShapeFromGradient{

protected:

public:
	static void run(const char* gradient_file, const char* output_path,
					 int refinement, int nurbs_order, NormType norm, BoundaryCondition bc);

};


#endif /* SHAPEFROMGRADIENT_H_ */
