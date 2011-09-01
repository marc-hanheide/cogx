/*
 * femell.h
 *
 *  Created on: Feb 16, 2010
 *      Author: jbalzer
 */

#ifndef FDMPOISSON2D_H_
#define FDMPOISSON2D_H_

#include "blztools/src/blztools.h"

class Poisson2d {

public:

	Poisson2d(JImg2d* grid);  // in here is right side

	// set routines
	void addDirichlet(int i, int j, double d) { m_Dirichlet.insert(std::pair<int,double>(this->A(i,j),d)); };
	bool isDirichlet(int i, int j) { return m_Dirichlet.find(this->A(i,j))!=m_Dirichlet.end(); };
	void addNeumann(int i, int j, cvec n) { m_Neumann.insert(std::pair<int,cvec>(this->A(i,j),n)); };					// n contains direction and value
	bool isNeumann(int i, int j) { return m_Neumann.find(this->A(i,j))!=m_Neumann.end(); };
	void addHomogenousDirichlet(double offset = 0);
	void addHomogenousNeumann(double offset = 1);

	// solve
	bool solve();

	bool save(const char* fileName);

private:

	enum { NORTH = 1, NORTHEAST = 2, EAST = 3, SOUTHEAST = 4, SOUTH = 5, SOUTHWEST = 6, WEST = 7, NORTHWEST = 8 };

	JImg2d* m_grid;

	std::map<int,double> m_Dirichlet;
	std::map<int,cvec> m_Neumann;

	bool assembleStiffnessMatrix(cholmod_sparse* K, bool normalizing = false);

	int A(int i, int j) { return m_grid->shape(1)*i + j; };
	int loc(int i, int j);
	bool isBoundary(int i, int j) { return (i==0 || j==0 || i==(m_grid->shape(0)-1) || j==(m_grid->shape(1)-1)); }

};


#endif /* FDMPOISSON2D_H_ */
