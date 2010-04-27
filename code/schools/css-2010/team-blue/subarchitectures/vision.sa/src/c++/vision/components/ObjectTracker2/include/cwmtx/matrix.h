// -*-c++-*-
#ifndef IG_MATRIX_H
#define IG_MATRIX_H

// $Id: matrix.h 184 2008-05-10 19:43:17Z hkuiper $

// CwMtx matrix and vector math library
// Copyright (C) 1999-2001  Harry Kuiper
// Copyright (C) 2000  Will DeVore (template conversion)

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
// USA

#include <iostream>
#include <iomanip>

// CWMatrix class

// This library was designed to mirror as closely as possible the
// notation used in mathematical writing. A matrix is indexed:
// matMatrixName[row][col].

// CAUTION!!!

// This matrix library was implemented with emphasis on
// speed. Consequently no attempts were made to trap and report
// errors. It is left entirely to the user to write code that does not
// cause errors within the matrix routines.

namespace CwMtx
{
  using std::ostream;

  // matrix status values
  enum { N_NOTALLOCATED, N_ALLOCATED, N_MAPPED };

  // Classes that create unity and zero "objects".  The ones directly
  // below work only for template arguments that are basic (numerical)
  // types that can be initialised by a literal 0 or 1.
  template <class T> class CWTUnity
  {
  public:
    operator T() { return 1; }
  };

  template <class T> class CWTZero
  {
  public:
    operator T() { return 0; }
  };

  // This template defaults to double. Most of the time this template
  // will be working with math functions that only work with
  // doubles. For example, the transcendental function sin(x) takes
  // and returns a double which would force the compiler to convert
  // back and forth from some other data type to a double.

  // prefix mat
  template < class T = double >
  class CWTMatrix
  {
  public:
    typedef T element;

    // creates a matrix, does NOT allocate rows and columns
    CWTMatrix();
    // creates a matrix, allocates rows and columns
    CWTMatrix(unsigned, unsigned);
    CWTMatrix(const CWTMatrix &);
    // sub-matrix mapped into another
    CWTMatrix(const CWTMatrix &, unsigned, unsigned, unsigned, unsigned);
	
    // removes matrix elements from free store
    ~CWTMatrix() { deallocate(); };

    // allocates rows and colums
    void dimension(unsigned, unsigned);
    // maps matrix into another
    void mapInto(const CWTMatrix&, unsigned, unsigned, unsigned, unsigned);
    // reverses the effect of dimension() and mapInto()
    void deallocate();

    int getStatus() const { return m_nMatStatus; };
    unsigned getRows() const { return m_crow; };
    unsigned getCols() const { return m_ccol; };

    // basic matrix operations

    // returns a row of modifyable elements
    T* operator [](unsigned irow) { return m_rgrow[irow]; };
    // returns a row of non-modifyable elements
    const T* operator [](unsigned irow) const { return m_rgrow[irow]; };

    CWTMatrix operator +(const CWTMatrix &) const;
    CWTMatrix operator -(const CWTMatrix &) const;
    CWTMatrix operator -() const;
    CWTMatrix operator *(const T &) const;
    CWTMatrix operator *(const CWTMatrix &) const;

    // Interesting note here. Because we are defining the "/" operator
    // we can't expect to use it inside the definition unless we
    // safely restrict it to operating on constants. Without the
    // parens the operator does the "* 1" first then does the
    // "/value" which then leads to calling the "/" etc...  With the
    // parens, the intended scalar operation, "1/value", occurs
    // first then the Matrix/Scalar operations occur.  If the parens
    // are missing in the operator below, an ifinite loop
    // occurs. -----------------------------------------------V----------V
    CWTMatrix operator /(const T &value) const
    {
      // NOTE: This used to work with g++ <= 3.2.
      // return (*this)*static_cast<const T &>(CWTUnity<T>()/value);

      T tTmp = CWTUnity<T>();
      tTmp /= value;
      return (*this)*tTmp;
    }

    // not inherited
    CWTMatrix & operator =(const CWTMatrix &);
    CWTMatrix & operator +=(const CWTMatrix &);
    CWTMatrix & operator -=(const CWTMatrix &);
    CWTMatrix & operator *=(const T &);
    CWTMatrix & operator /=(const T &value)
    {
      // NOTE: This used to work with g++ <= 3.2.
      // return (*this) *= static_cast<const T &>(CWTUnity<T>()/value);

      T tTmp = CWTUnity<T>();
      tTmp /= value;
      return (*this) *= tTmp;
    }

    int operator ==(const CWTMatrix &) const;
    int operator !=(const CWTMatrix &mat) const { return !( (*this) == mat ); }

    // stores CWMatrix + CWMatrix in this
    void storeSum(const CWTMatrix &, const CWTMatrix &);
    // stores CWMatrix*CWMatrix in this
    void storeProduct(const CWTMatrix &, const CWTMatrix &);
    // stores transpose of CWMatrix in this
    void storeTranspose(const CWTMatrix &);
    // stores CWMatrix at indicated position in this
    void storeAtPosition(unsigned, unsigned, const CWTMatrix &);
    // fills the whole array with a value.
    void fill(const T &);

    void interchangeRows(unsigned, unsigned);
    void addRowToRow(unsigned, unsigned, const T & = CWTUnity<T>());
    void multiplyRow(unsigned, const T &);

  private:
    // initializes data members
    void initialize();

    // we keep the data structures used for CWMatrix implementation
    // private

    // row count
    unsigned m_crow;
    // column count
    unsigned m_ccol;
    // an array of rows (stored on free store)
    T **m_rgrow;
    // matrix status
    int m_nMatStatus;
  };

  // Templates to create self-dimensioning CWTMatrix classes - or one
  // of its derived classes - using the syntax of a default
  // constructor.  This facility is required for using matrices as
  // elements of matrices since these will always be created by a call
  // to the default constructor.

  template <class T, unsigned crow, unsigned ccol>
  class CWTMat: public T
  {
  public:
    CWTMat(): T(crow, ccol) {}

    T & operator =(const T &mtx) { return T::operator=(mtx); }
  };

  // NOTE: There exists no unity matrix for a non-square matrix!

  // Zero matrix.  NOTE: A zero matrix can only be constructed for a
  // matrix of known dimensions.  Hence the use of CWTMat<T,n,m>.
  template <class T, unsigned crow, unsigned ccol>
  class CWTZero< CWTMat<CWTMatrix<T>, crow, ccol> >:
    public CWTMat<CWTMatrix<T>, crow, ccol>
  {
  public:
    CWTZero() { fill(CWTZero<T>()); }
  };

  //
  // Constructors
  //

  template < class T >
  inline CWTMatrix<T>::CWTMatrix()
  {
    initialize();
  };

  // creates a matrix, allocates rows and columns
  template < class T >
  inline CWTMatrix<T>::CWTMatrix(unsigned crow, unsigned ccol)
  {
    initialize();
    dimension(crow, ccol);
  }

  template < class T >
  inline CWTMatrix<T>::CWTMatrix(const CWTMatrix<T> &mat)
  {
    initialize();

    if (mat.m_nMatStatus == N_NOTALLOCATED)
      {
	// input matrix not allocated, so there's nothing to copy
	return;
      }
    else
      {
	// copy contents of input matrix
	(*this) = mat;
      }
  }

  // Mapped matrix constructor
  template < class T >
  inline CWTMatrix<T>::CWTMatrix(const CWTMatrix<T> &mat,
				 unsigned irowStart,
				 unsigned icolStart,
				 unsigned irowEnd,
				 unsigned icolEnd)
  {
    initialize();
    mapInto(mat, irowStart, icolStart, irowEnd, icolEnd);
  }

  //
  // Private Methods
  //

  // initial values for matrix attributes
  template < class T >
  inline void CWTMatrix<T>::initialize()
  {
    m_crow       = 0;
    m_ccol       = 0;
    m_rgrow      = NULL;
    m_nMatStatus = N_NOTALLOCATED;
  }

  //
  // User Methods
  //

  template < class T >
  void CWTMatrix<T>::dimension(unsigned crowInit, unsigned ccolInit)
  {
    if (m_nMatStatus != N_NOTALLOCATED)
      {
	deallocate();
      }

    m_crow = crowInit;
    m_ccol = ccolInit;

#ifdef CC_CWTMTX_ASSUME_BASIC_TYPES
    // NOTE: CWTMatrix normally uses the standard C++ new() operator
    // to allocate memory for matrix elements.  Using malloc(3) can
    // save time because it can allocate all required memory in a
    // single call.  However, malloc(3) only works for template
    // arguments that are C++ basic types because basic types are no
    // classes and thus need not be initialised by a constructor.  To
    // use malloc(3) #define CC_CWTMTX_ASSUME_BASIC_TYPES in all
    // source files that use CWTMatrix templates before you #include
    // them.

    // Allocate space for row pointers and the rows themselves using
    // ANSI C malloc(3) function.
    m_rgrow = reinterpret_cast<T **>(malloc(m_crow*sizeof(T *)
					    + m_crow*m_ccol*sizeof(T)));
    T *ptTmp = reinterpret_cast<T *>(&(m_rgrow[m_crow]));
#else
    // Allocate space for row pointers and the rows themselves using
    // ANSI C++ new() operator.
    m_rgrow = new T*[m_crow];
    T *ptTmp = new T[m_crow*m_ccol];
#endif

    // make row pointers point to start of each row
    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	m_rgrow[irow] = &(ptTmp[irow*m_ccol]);
      }

    m_nMatStatus = N_ALLOCATED;
  }

  // maps a matrix into another matrix, deallocates first if neccesary
  // allocates space on the free store for a matrix, deallocates first
  // if neccesary
  template < class T >
  void CWTMatrix<T>::mapInto(const CWTMatrix<T> &mat,
			     unsigned irowStart,
			     unsigned icolStart,
			     unsigned irowEnd,
			     unsigned icolEnd )
  {
    if (m_nMatStatus != N_NOTALLOCATED)
      {
	deallocate();
      }

    // calculate columns
    m_crow = irowEnd - irowStart + 1;

    // calculate rows
    m_ccol = icolEnd - icolStart + 1;

    // allocate space for row pointers
#ifdef CC_CWTMTX_ASSUME_BASIC_TYPES
    m_rgrow = reinterpret_cast<T **>(malloc(m_crow*sizeof(T *)));
#else
    m_rgrow = new T*[m_crow];
#endif

    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	// get values for row pointers
	m_rgrow[irow] = &mat.m_rgrow[irow + irowStart][icolStart];
      }

    m_nMatStatus = N_MAPPED;
  }

  // deallocates a matrix' elements from the free store
  template < class T >
  void CWTMatrix<T>::deallocate()
  {
    // has to take account of the state the matrix is in
    switch (m_nMatStatus)
      {
      case N_NOTALLOCATED:
	// nothing needs to be deallocated
	break;

      case N_MAPPED:
	// delete the array of row pointers
#ifdef CC_CWTMTX_ASSUME_BASIC_TYPES
	free(m_rgrow);
#else
	delete [] m_rgrow;
#endif
	break;

      case N_ALLOCATED:
	//   delete the contents of the matrix completely
#ifdef CC_CWTMTX_ASSUME_BASIC_TYPES
	// Deallocate space for row pointers and the rows themselves
	// using ANSI C free(3) function.
	free(m_rgrow);
#else
	// Deallocate space for row pointers and the rows themselves
	// using ANSI C++ delete() operator.
	delete [] *m_rgrow;
	delete [] m_rgrow;
#endif
	break;
      };

    // bring this matrix in initialized state
    initialize();
  }

  template < class T >
  CWTMatrix<T> 
  CWTMatrix<T>::operator +(const CWTMatrix<T> &mat) const
  {
    // copy this and add argument
    return CWTMatrix<T>( *this ) += mat;
  }

  template < class T >
  CWTMatrix<T> 
  CWTMatrix<T>::operator -(const CWTMatrix<T> &mat) const
  {
    // copy this and subtract argument
    return CWTMatrix<T>( *this ) -= mat;
  }

  template < class T >
  CWTMatrix<T> CWTMatrix<T>::operator -() const
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this)*static_cast<const T &>(CWTZero<T>() - CWTUnity<T>());

    T tTmp  = CWTZero<T>();
    tTmp -= CWTUnity<T>();
    return (*this)*tTmp;
  }

  template < class T >
  CWTMatrix<T> CWTMatrix<T>::operator *(const T &value) const
  {
    // copy this and multiply by argument
    return CWTMatrix<T>( *this ) *= value;
  }

  template < class T >
  CWTMatrix<T>
  CWTMatrix<T>::operator *(const CWTMatrix<T> &mat) const
  {
    // create result matrix
    CWTMatrix matResult( m_crow , mat.m_ccol );
    // store (*this)*mat in result matrix
    matResult.storeProduct( *this , mat );
    return matResult;
  }

  // assignment operator
  template < class T >
  CWTMatrix<T> & 
  CWTMatrix<T>::operator =(const CWTMatrix<T> &mat)
  {
    if (m_nMatStatus == N_NOTALLOCATED)
      {
	// if this is not allocated, we'll allocate it on the fly
	dimension(mat.m_crow, mat.m_ccol);
      }

    // Copy the values from mat to this, excess values in mat are
    // ignored. If mat has too few elements, garbage will be copied
    // into remaining elements of this
    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	for (unsigned icol = 0; icol < m_ccol; ++icol)
	  {
	    m_rgrow[irow][icol] = mat.m_rgrow[irow][icol];
	  }
      }

    return *this;
  }

  template < class T >
  CWTMatrix<T> & 
  CWTMatrix<T>::operator +=(const CWTMatrix<T> &mat)
  {
    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	for (unsigned icol = 0; icol < m_ccol; ++icol)
	  {
	    m_rgrow[irow][icol] += mat.m_rgrow[irow][icol];
	  }
      }

    return *this;
  }

  template < class T >
  CWTMatrix<T> & 
  CWTMatrix<T>::operator -=(const CWTMatrix<T> &mat)
  {
    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	for (unsigned icol = 0; icol < m_ccol; ++icol)
	  {
	    m_rgrow[irow][icol] -= mat.m_rgrow[irow][icol];
	  }
      }

    return *this;
  }

  template < class T >
  CWTMatrix<T> & CWTMatrix<T>::operator *=(const T &value)
  {
    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	for (unsigned icol = 0; icol < m_ccol; ++icol)
	  {
	    m_rgrow[irow][icol] *= value;
	  }
      }

    return *this;
  }

  template < class T >
  int CWTMatrix<T>::operator ==(const CWTMatrix<T> &mat) const
  {
    if ((m_crow == mat.m_crow) && (m_ccol == mat.m_ccol))
      {
	for (unsigned irow = 0; irow < m_crow; ++irow)
	  {
	    for (unsigned icol = 0; icol < m_ccol; ++icol)
	      {
		if (m_rgrow[irow][icol] != mat.m_rgrow[irow][icol])
		  {
		    return 0;
		  }
	      }
	  }

	return 1;
      }
    else
      {
	return 0;
      }
  }

  // stores mat1 + mat2 in this
  template < class T >
  void CWTMatrix<T>::storeSum(const CWTMatrix<T> &mat1,
			      const CWTMatrix<T> &mat2)
  {
    // NOTE: it is assumed that this has correct dimensions,
    // i.e. CWMatrix(mat1.m_crow, mat2.m_ccol)

    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	for (unsigned icol = 0; icol < m_ccol; ++icol)
	  {
	    m_rgrow[irow][icol] = 
	      mat1.m_rgrow[irow][icol] + mat2.m_rgrow[irow][icol];
	  }
      }
  }

  // stores mat1*mat2 in this
  template < class T >
  void CWTMatrix<T>::storeProduct(const CWTMatrix<T> &mat1,
				  const CWTMatrix<T> &mat2)
  {
    // NOTE: it is assumed that this has correct dimensions,
    // i.e. CWMatrix(mat1.m_crow, mat2.m_ccol)

    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	for (unsigned icol = 0; icol < m_ccol; ++icol)
	  {
	    m_rgrow[irow][icol] = CWTZero<T>();

	    for (unsigned icol2 = 0; icol2 < mat1.m_ccol; ++icol2)
	      {
		m_rgrow[irow][icol] += 
		  mat1.m_rgrow[irow][icol2]*mat2.m_rgrow[icol2][icol];
	      }
	  }
      }
  }

  // mat should fit in this
  template < class T >
  void CWTMatrix<T>::storeAtPosition(unsigned irowStart,
				     unsigned icolStart,
				     const CWTMatrix<T> &mat)
  {
    for (unsigned irow = 0; irow < mat.m_crow; ++irow)
      {
	for (unsigned icol = 0; icol < mat.m_ccol; ++icol)
	  {
	    m_rgrow[irow + irowStart][icol + icolStart] =
	      mat.m_rgrow[irow][icol];
	  }
      }
  }

  // stores transpose of mat in this
  template < class T >
  void CWTMatrix<T>::storeTranspose(const CWTMatrix<T> &mat)
  {
    // NOTE: it is assumed that this has correct dimensions,
    // i.e. CWMatrix(mat.m_ccol, mat.m_crow)
    for (unsigned irow = 0; irow < m_crow; ++irow)
      {
	for (unsigned icol = 0; icol < m_ccol; ++icol)
	  {
		m_rgrow[irow][icol] = mat.m_rgrow[icol][irow];
	  }
      }
  }

  template < class T >
  inline void CWTMatrix<T>::fill(const T &elemFill)
  {
    unsigned iEnd = m_crow*m_ccol;

    for (unsigned i = 0; i < iEnd; ++i)
      {
	(*m_rgrow)[i] = elemFill;
      }
  }

  template < class T >
  void CWTMatrix<T>::interchangeRows(unsigned irow1, unsigned irow2)
  {
    T* rowSav      = m_rgrow[irow1];

    // switch row pointers
    m_rgrow[irow1] = m_rgrow[irow2];
    m_rgrow[irow2] = rowSav;
  }

  template < class T >
  void CWTMatrix<T>::multiplyRow(unsigned irow, const T &value)
  {
    for (unsigned icol = 0; icol < m_ccol; ++icol)
      {
	m_rgrow[irow][icol] *= value;
      }
  }

  template < class T >
  void CWTMatrix<T>::addRowToRow(unsigned irowSrc,
				 unsigned irowDest,
				 const T &value)
  {
    for (unsigned icol = 0; icol < m_ccol; ++icol)
      {
	m_rgrow[irowDest][icol] += m_rgrow[irowSrc][icol]*value;
      }
  }

  //
  // template functions designed work with the base matrix class.
  //

  template < class T >
  inline CWTMatrix<T> operator *(const T &value,
				 const CWTMatrix<T> &mat)
  {
    return mat*value;
  }

  template < class T >
  CWTMatrix<T> transpose(const CWTMatrix<T> &mat)
  {
    CWTMatrix<T> matTranspose( mat.getCols(), mat.getRows() );
    matTranspose.storeTranspose(mat);
    return matTranspose;
  }

  template < class T >
  ostream & operator <<(ostream &os, const CWTMatrix<T>& mtx)
  {
    os << "[" ;

    for (unsigned i = 0; i < mtx.getRows(); i++)
      {
	if (i > 0)
	  {
	    os << "; ";
	  }

        os  << mtx[i][0];

	for (unsigned j = 1; j < mtx.getCols(); j++)
	  {
	    os  << ", " << mtx[i][j];
	  }
      }

    os << "]";

    return os;
  }
}

#endif // IG_MATRIX_H

