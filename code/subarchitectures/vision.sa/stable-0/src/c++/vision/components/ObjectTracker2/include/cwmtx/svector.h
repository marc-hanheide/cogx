// -*-c++-*-
#ifndef IG_SVECTOR_H
#define IG_SVECTOR_H

// $Id: svector.h 184 2008-05-10 19:43:17Z hkuiper $

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

#ifndef IG_VECTOR_H
#include "vector.h"
#endif

#ifndef IG_SMATRIX_H
#include "smatrix.h"
#endif

namespace CwMtx
{

  // prefix svec
  template < class T = double >
  class CWTSpaceVector: public CWTVector<T>
  {
  public:
    CWTSpaceVector(): CWTVector<T>(3U) {};
    CWTSpaceVector(const CWTMatrix<T> &mat): CWTVector<T>(mat) {};
    CWTSpaceVector(const CWTVector<T> &vec): CWTVector<T>(vec) {};
    CWTSpaceVector(const CWTSpaceVector &svec): CWTVector<T>(svec) {};
    // construct from 3 elements
    CWTSpaceVector(const T &, const T &, const T &);
    CWTSpaceVector(const CWTMatrix<T> &, unsigned, unsigned);
    CWTSpaceVector(const CWTVector<T> &vec,
		   unsigned irowStart): CWTVector<T>(vec,
						     irowStart,
						     irowStart + 2) {};

    ~CWTSpaceVector() {};

    void dimension() { CWTVector<T>::dimension(3); };
    void mapInto(const CWTMatrix<T> &mat,
		 unsigned irowStart,
		 unsigned icolStart);
    void mapInto(const CWTVector<T> &vec,
		 unsigned irowStart = 0);

    CWTSpaceVector operator +(const CWTSpaceVector &) const;
    CWTSpaceVector operator -(const CWTSpaceVector &) const;
    CWTSpaceVector operator -() const;
    CWTSpaceVector operator *(const T &) const;
    // inner product
    T operator *(const CWTSpaceVector &) const;
    // outer product
    CWTSpaceVector operator %(const CWTSpaceVector &) const;
    CWTSpaceVector operator /(const T &value) const;

    // not inherited
    CWTSpaceVector & operator =(const CWTSpaceVector &);
    CWTSpaceVector & operator +=(const CWTSpaceVector &);
    CWTSpaceVector & operator -=(const CWTSpaceVector &);
    CWTSpaceVector & operator *=(const T &);
    // outer product
    CWTSpaceVector & operator %=(const CWTSpaceVector &);
    CWTSpaceVector & operator /=(const T &value);

    void storeOuterProduct(const CWTSpaceVector &, const CWTSpaceVector &);

    // returns a unit vector with same direction as this
    CWTSpaceVector unit() const { return (*this)/(this->norm()); }
  };

  // NOTE: There exists no unity space vector!

  // Zero space vector.
  template <class T>
  class CWTZero< CWTSpaceVector<T> >: public CWTSpaceVector<T>
  {
  public:
    CWTZero() { fill(CWTZero<T>()); }
  };

  //
  // Constructors
  //

  template < class T >
  inline CWTSpaceVector<T>::CWTSpaceVector(const CWTMatrix<T> &mat,
					   unsigned irowStart,
					   unsigned icolStart)
    :
    CWTVector<T>(mat, irowStart, icolStart, irowStart + 2)
  {
  }

  template < class T >
  inline CWTSpaceVector<T>::CWTSpaceVector(const T &elem1,
					   const T &elem2,
					   const T &elem3)
    :
    CWTVector<T>(3U)
  {
    (*this)[0] = elem1;
    (*this)[1] = elem2;
    (*this)[2] = elem3;
  }

  //
  // User Methods
  //

  template < class T >
  inline void CWTSpaceVector<T>::mapInto(const CWTMatrix<T> &mat,
					 unsigned irowStart,
					 unsigned icolStart)
  {
    CWTVector<T>::mapInto(mat, irowStart, icolStart, irowStart + 2);
  }

  template < class T >
  inline void CWTSpaceVector<T>::mapInto(const CWTVector<T> &vec,
					 unsigned irowStart)
  {
    CWTVector<T>::mapInto(vec, irowStart, irowStart + 2);
  }

  template < class T >
  inline CWTSpaceVector<T> CWTSpaceVector<T>::operator /(const T &value) const
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this)*static_cast<const T &>(CWTUnity<T>()/value);

    T tTmp = CWTUnity<T>();
    tTmp /= value;
    return (*this)*tTmp;
  }

  // not inherited
  template < class T >
  inline CWTSpaceVector<T> &
  CWTSpaceVector<T>::operator =(const CWTSpaceVector<T> &svec)
  {
    return static_cast<CWTSpaceVector &>(CWTMatrix<T>::operator=(svec));
  }

  template < class T >
  inline CWTSpaceVector<T> &
  CWTSpaceVector<T>::operator +=(const CWTSpaceVector<T> &svec)
  {
    return static_cast<CWTSpaceVector &>(CWTMatrix<T>::operator+=(svec));
  }

  template < class T >
  inline CWTSpaceVector<T> &
  CWTSpaceVector<T>::operator -=(const CWTSpaceVector<T> &svec)
  {
    return static_cast<CWTSpaceVector &>(CWTMatrix<T>::operator-=(svec));
  }

  template < class T >
  inline CWTSpaceVector<T> & CWTSpaceVector<T>::operator *=(const T &value)
  {
    return static_cast<CWTSpaceVector &>(CWTMatrix<T>::operator*=(value));
  }

  template < class T >
  inline CWTSpaceVector<T> & CWTSpaceVector<T>::operator /=(const T &value)
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this) *= static_cast<const T &>(CWTUnity<T>()/value);

    T tTmp = CWTUnity<T>();
    tTmp /= value;
    return (*this) *= tTmp;
  }

  // outer product
  template < class T >
  inline CWTSpaceVector<T> &
  CWTSpaceVector<T>::operator %=(const CWTSpaceVector<T> &vec)
  {
    return (*this) = (*this)%vec;
  }

  template < class T >
  CWTSpaceVector<T>
  CWTSpaceVector<T>::operator +(const CWTSpaceVector<T> &svec) const
  {
    return CWTSpaceVector<T>(*this) += svec;
  }

  template < class T >
  CWTSpaceVector<T>
  CWTSpaceVector<T>::operator -(const CWTSpaceVector<T> &svec) const
  {
    return CWTSpaceVector<T>(*this) -= svec;
  }

  template < class T >
  CWTSpaceVector<T> CWTSpaceVector<T>::operator -() const
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this)*static_cast<const T &>(CWTZero<T>() - CWTUnity<T>());

    T tTmp = CWTZero<T>();
    tTmp -= CWTUnity<T>();
    return (*this)*tTmp;
  }

  template < class T >
  CWTSpaceVector<T> CWTSpaceVector<T>::operator *(const T &value) const
  {
    return CWTSpaceVector<T>(*this) *= value;
  }

  // inner product
  template < class T >
  T CWTSpaceVector<T>::operator *(const CWTSpaceVector<T> &vec) const
  {
    return CWTVector<T>::operator*(vec);
  }

  template < class T >
  void CWTSpaceVector<T>::
  storeOuterProduct(const CWTSpaceVector<T> &svecLeft,
		    const CWTSpaceVector<T> &svecRight)
  {
    // to reduce the number of calls to the subscript operator
    T svecLeft0 = svecLeft[0];
    T svecLeft1 = svecLeft[1];
    T svecLeft2 = svecLeft[2];

    T svecRight0 = svecRight[0];
    T svecRight1 = svecRight[1];
    T svecRight2 = svecRight[2];

    (*this)[0] = svecLeft1*svecRight2 - svecLeft2*svecRight1;
    (*this)[1] = svecLeft2*svecRight0 - svecLeft0*svecRight2;
    (*this)[2] = svecLeft0*svecRight1 - svecLeft1*svecRight0;
  }

  template < class T >
  CWTSpaceVector<T>
  CWTSpaceVector<T>::operator %(const CWTSpaceVector<T> &svec) const
  {
    CWTSpaceVector<T> svecResult;
    svecResult.storeOuterProduct(*this, svec);
    return svecResult;
  }

  //
  // template functions designed work with the base matrix class.
  //

  template < class T >
  inline CWTSpaceVector<T> operator *(const T &value,
				      const CWTSpaceVector<T> &svec)
  {
    return svec*value;
  }

  // (square matrix)*(space vector) must yield a space vector
  template < class T >
  CWTSpaceVector<T> operator *(const CWTSquareMatrix<T> &smat,
			       const CWTSpaceVector<T> &svec)
  {
    CWTSpaceVector<T> svecResult;
    svecResult.storeProduct(smat, svec);
    return svecResult;
  }

  // the sign of a vector is a unit vector with the same direction
  template <class T>
  inline CWTSpaceVector<T>
  sgn(const CWTSpaceVector<T> &svec)
  {
    return svec.unit();
  }
}

#endif // IG_SVECTOR_H
