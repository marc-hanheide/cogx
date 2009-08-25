// -*-c++-*-
#ifndef IG_VECTOR_H
#define IG_VECTOR_H

// $Id: vector.h 165 2008-01-19 19:53:19Z hkuiper $

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

#include <cmath>

#ifndef IG_MATRIX_H
#include "matrix.h"
#endif

namespace CwMtx
{

  template < class T = double >
  class CWTVector : public CWTMatrix<T>
  {
  public:
    CWTVector(): CWTMatrix<T>() {};
    CWTVector(unsigned crowInit): CWTMatrix<T>(crowInit, 1) {};
    CWTVector(const CWTMatrix<T>& mat): CWTMatrix<T>(mat) {};
    CWTVector(const CWTVector& vec): CWTMatrix<T>(vec) {};
    // mapping into a matrix
    CWTVector(const CWTMatrix<T>& , unsigned, unsigned, unsigned);
    // mapping into a vector
    CWTVector(const CWTVector& , unsigned, unsigned);

    ~CWTVector() {};

    void mapInto(const CWTMatrix<T> &, unsigned, unsigned, unsigned);
    void mapInto(const CWTVector &, unsigned, unsigned);
    void dimension(unsigned crowInit)
    {
      CWTMatrix<T>::dimension(crowInit, 1);
    }

    T & operator [](unsigned irow)
    {
      return this->CWTMatrix<T>::operator[](irow)[0];
    }

    const T & operator [](unsigned irow) const
    {
      return this->CWTMatrix<T>::operator[](irow)[0];
    }

    CWTVector operator +(const CWTVector &) const;
    CWTVector operator -(const CWTVector &) const;
    CWTVector operator -() const;
    CWTVector operator *(const T &) const;
    // CWTVector*CWTVector, inner product
    T operator *(const CWTVector &) const;
    CWTVector operator /(const T &value) const
    {
      // NOTE: This used to work with g++ <= 3.2.
      // return (*this)*static_cast<const T &>(CWTUnity<T>()/value);

      T tTmp = CWTUnity<T>();
      tTmp /= value;
      return (*this)*tTmp;
    }

    // not inherited
    CWTVector & operator =(const CWTVector &vec);
    CWTVector & operator +=(const CWTVector &vec);
    CWTVector & operator -=(const CWTVector &vec);
    CWTVector & operator *=(const T &value);
    CWTVector & operator /=(const T &value);

    // CWTVector norm
    T operator !() const { return (*this).norm(); };

    void storeAtRow(unsigned, const CWTVector &);
    // returns vector norm (length)
    T norm() const;
    // returns a unit vector with same direction as this
    CWTVector unit() const { return (*this)/norm(); }

    // make this a unit vector
    void makeUnit() { (*this) /= norm(); }
  };

  template <class T, unsigned crow>
  class CWTVec: public T
  {
  public:
    CWTVec(): T(crow) {}

    T & operator =(const T &mtx) { return T::operator=(mtx); }
  };

  // NOTE: There exists no unity vector for a general vector!

  // Zero matrix.
  template <class T, unsigned crow>
  class CWTZero< CWTVec<CWTVector<T>, crow> >:
    public CWTVec<CWTVector<T>, crow>
  {
  public:
    CWTZero() { fill(CWTZero<T>()); }
  };

  //
  // Constructors
  //

  // mapping into a vector
  template < class T >
  inline CWTVector<T>::CWTVector(const CWTVector<T> &vec,
				 unsigned irowStart,
				 unsigned irowEnd)
    : CWTMatrix<T>(vec, irowStart, 0, irowEnd, 0)
  {
  }

  // mapping into a matrix
  template < class T >
  inline CWTVector<T>::CWTVector(const CWTMatrix<T> &mat,
				 unsigned irowStart,
				 unsigned icolStart,
				 unsigned irowEnd)
    :
    CWTMatrix<T>(mat, irowStart, icolStart, irowEnd, icolStart)
  {
  }

  //
  // User Methods
  //

  template < class T >
  inline void CWTVector<T>::mapInto(const CWTMatrix<T> &mat,
				    unsigned irowStart,
				    unsigned icol,
				    unsigned irowEnd)
  {
    CWTMatrix<T>::mapInto(mat, irowStart, icol, irowEnd, icol);
  }

  template < class T >
  inline void CWTVector<T>::mapInto(const CWTVector &vec,
				    unsigned irowStart,
				    unsigned irowEnd)
  {
    CWTMatrix<T>::mapInto(vec, irowStart, 0, irowEnd, 0);
  }

  // not inherited
  template < class T >
  inline CWTVector<T> & CWTVector<T>::operator =(const CWTVector<T> &vec)
  {
    return static_cast<CWTVector &>(CWTMatrix<T>::operator=(vec));
  }

  template < class T >
  inline CWTVector<T> & CWTVector<T>::operator +=(const CWTVector<T> &vec)
  {
    return static_cast<CWTVector &>(CWTMatrix<T>::operator+=(vec));
  }

  template < class T >
  inline CWTVector<T> & CWTVector<T>::operator -=(const CWTVector<T> &vec)
  {
    return static_cast<CWTVector &>(CWTMatrix<T>::operator-=(vec));
  }

  template < class T >
  inline CWTVector<T> & CWTVector<T>::operator *=(const T &value)
  {
    return static_cast<CWTVector &>(CWTMatrix<T>::operator*=(value));
  }

  template < class T >
  inline CWTVector<T> & CWTVector<T>::operator /=(const T &value)
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this) *= static_cast<const T &>(CWTUnity<T>()/value);

    T tTmp = CWTUnity<T>();
    tTmp /= value;
    return (*this) *= tTmp;
  }

  template < class T >
  inline void CWTVector<T>::storeAtRow(unsigned irowStart,
				       const CWTVector<T> &vec)
  {
    CWTMatrix<T>::storeAtPosition(irowStart, 0, vec);
  }

  template < class T >
  CWTVector<T> CWTVector<T>::operator +(const CWTVector<T> &vec) const
  {
    return CWTVector<T>(*this) += vec;
  }

  template < class T >
  CWTVector<T> CWTVector<T>::operator -(const CWTVector<T> &vec) const
  {
    return CWTVector<T>(*this) -= vec;
  }

  template < class T >
  CWTVector<T> CWTVector<T>::operator -() const
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this)*static_cast<const T &>(CWTZero<T>() - CWTUnity<T>());

    T tTmp = CWTZero<T>();
    tTmp -= CWTUnity<T>();
    return (*this)*tTmp;
  }

  template < class T >
  CWTVector<T> CWTVector<T>::operator *(const T &value) const
  {
    return CWTVector<T>(*this) *= value;
  }

  template < class T >
  T CWTVector<T>::operator *(const CWTVector<T> &vec) const
  {
    T elemResult = CWTZero<T>();

    for (unsigned irow = 0; irow < (*this).getRows(); ++irow)
      {
	elemResult += (*this)[irow]*vec[irow];
      }

    return elemResult;
  }

  // length of vector
  template < class T >
  T CWTVector<T>::norm() const
  {
    T elemResult = CWTZero<T>();

    elemResult = (*this)*(*this);

    return sqrt( elemResult );
  }

  //
  // template functions designed work with the vector class.
  //

  template < class T >
  inline CWTVector<T> operator *(const T &value, const CWTVector<T> &vec)
  {
    return vec*value;
  }

  // matrix*vector must yield a vector
  template < class T >
  CWTVector<T> operator *(const CWTMatrix<T> &mat, const CWTVector<T> &vec)
  {
    CWTVector<T> vecResult(mat.getRows());
    vecResult.storeProduct(mat, vec);
    return vecResult;
  }

  // norm computation as a function
  template < class T >
  inline T norm(const CWTVector<T> &vec)
  {
    return vec.norm();
  }

  // the sign of a vector is a unit vector with the same direction
  template <class T>
  inline CWTVector<T>
  sgn(const CWTVector<T> &vec)
  {
    return vec.unit();
  }
}

#endif // IG_VECTOR_H
