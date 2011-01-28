// ==================================================================
// libCRFH
// Copyright (C) 2008, 2009  Andrzej Pronobis
//
// This file is part of libCRFH.
//
// libCRFH is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libCRFH is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libCRFH. If not, see <http://www.gnu.org/licenses/>.
// ==================================================================

/**
 * \file CMatrix.h
 * \author Andrzej Pronobis
 *
 * Contains declaration of the CMatrix class.
 */

#ifndef _CMATRIX_H_
#define _CMATRIX_H_

#include <QtCore/QString>
#include <QtCore/QByteArray>



/**
* Header of a .mat file.
*/
struct CMatFileHeader
{
  long type;
  long mrows;
  long ncols;
  long imagf;
  long namelen;
};



/**
* A matrix of double values.
*/
class CMatrix
{

public:

  /** Default contructor. */
  inline CMatrix(): _data(0), _rows(0), _cols(0)
    {};

  /** Contructor. */
  CMatrix(int rows, int cols);

  /** Destructor. */
  ~CMatrix();


  /** Prints the contents of the matrix to the std output. */
  void print();


public:

  /** Resizes the matrix if the current size if different than given. */
  void resize(int rows, int cols);

  /** Returns true if the matrix is empty. */
  inline bool isEmpty() const
  {
    return !_data;
  }

  /** Returns the number of rows. */
  inline int getRows() const
  {
    return _rows;
  }

  /** Returns the number of cols. */
  inline int getCols() const
  {
    return _cols;
  }


public:

  /** Sets an element of the matrix. */
  inline void setElem(int row, int col, double value)
  {
    _data[row*_cols+col]=value;
  }

  /** Sets an element of the matrix. */
  inline void setElem(int elem, double value)
  {
    _data[elem]=value;
  }

  /** Returns an element of the matrix. */
  inline double getElem(int row, int col) const
  {
    return _data[row*_cols+col];
  }

  /** Returns an element of the matrix. */
  inline double getElem(int elem) const
  {
    return _data[elem];
  }

  /** Returns a pointer to the internal data. */
  inline double *getRawData() const
  {
    return _data;
  }


public:

  /** Division operator. */
  CMatrix &operator/=(double value);

  /** Multiplication operator. */
  CMatrix &operator*=(double value);

  /** Returns a transposed matrix. */
  CMatrix *getTransposed(CMatrix *result = 0) const;

  /** Returns a deep copy of the matrix. */
  CMatrix *getDeepCopy(CMatrix *result = 0) const;

public:

  /** Saves the contents of the as a Matlab mat file. */
  int saveToMatFile(QString fileName, QByteArray variableName);


public:

  /** Returns maximum value. */
  double getMax() const;

  /** Returns minimum value. */
  double getMin() const;

  /** Returns the sum of all elements. */
  double getSum() const;


public:

  /** Convolves the matrix with another matrix.
      The size of the matrix remains identical (as in case
      of matlabs conv2(,,'same'). */
  void convolveWith(const CMatrix &m);

  void convolveWithFft(const CMatrix &m);


public:

  /** Convolves two matrices. The size of the resulting
      matrix will be the same as the size of the matrix A.
     (as in case of matlabs conv2(,,'same'). */
  static CMatrix *convolve(const CMatrix &mA, const CMatrix &mB, CMatrix *mC = 0);

//  static CMatrix *convolveFft(const CMatrix &mA, const CMatrix &mB, CMatrix *mC = 0);

private:

  /** Data */
  double *_data;

  /** Rows */
  int _rows;

  /** Cols */
  int _cols;

};


#endif
