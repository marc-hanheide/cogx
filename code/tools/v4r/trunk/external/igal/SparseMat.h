/*
 * SparseMat.h
 *
 *  Created on: Mar 19, 2011
 *      Author: moerwald
 */

#ifndef SPARSEMAT_H_
#define SPARSEMAT_H_

#include <vector>
#include <map>
#include <stdio.h>

class SparseVector{
public:
	std::vector<int> m_i;	// column numbers of the row
	std::vector<double> m_v; // contains the values of the row

	// read access operators
	double operator[](unsigned i) const;
	double at(unsigned i) const;

	// write access operators
	void set(int i, double val);

	// negate values of row
	const SparseVector operator-() const;

	// dump values of row to screen
	void dump();

	// special lp_solve function
	std::vector<int> colno_plus(int v);

};


class SparseRowMat : public std::vector<SparseVector>
{
public:
	SparseRowMat();

	inline void set(int row, int col, double val){ this->at(row).set(col, val); }

	inline unsigned getCols(){ return cols; }
	inline unsigned getRows(){ return this->size(); }

	inline void setCols(unsigned c){ cols = c; }
	void setCols();

	void save(const char* filename, int zp=0); // save with zero-padding

protected:
	unsigned cols;
};

class SparseMat{
protected:
	std::map< int, std::map<int,double> > m_mat;

public:
	void get(std::vector<int> &i, std::vector<int> &j, std::vector<double> &v);
	double get(int i, int j);
	void set(int i, int j, double v);

	void deleteRow(int i);
	void deleteColumn(int j);

	inline void clear(){ m_mat.clear(); }
	void size(int &si, int &sj);
	int nonzeros();
	void printLong();
	void print();

};

#endif /* SPARSEMAT_H_ */
