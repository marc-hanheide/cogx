// -*-c++-*-
#ifndef IG_QUATERN_H
#define IG_QUATERN_H

// $Id: quatern.h 184 2008-05-10 19:43:17Z hkuiper $

// CwMtx matrix and vector math library
// Copyright (C) 1999-2001  Harry Kuiper
// Copyright (C) 2000  Will DeVore (template conversion)
// Copyright (C) 2000  Jiri Ecer (constructor from exponential form)

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

#ifndef IG_SVECTOR_H
#include "svector.h"
#endif

namespace CwMtx
{
  using std::exp;
  using std::log;

  // prefix qtn
  template < class T = double >
  class CWTQuaternion : public CWTVector<T>
  {
  public:
	
    CWTQuaternion(): CWTVector<T>(4U) {};
    // assumption: mat is column vector with four elements
    CWTQuaternion(const CWTMatrix<T> &mat): CWTVector<T>(mat) {};
    // assumption: vec has four elements
    CWTQuaternion(const CWTVector<T> &vec): CWTVector<T>(vec) {};
    CWTQuaternion(const CWTQuaternion &qtn): CWTVector<T>(qtn) {};
    // the space vector will become the quaternion's imaginary part, T
    // will become its real part
    CWTQuaternion(const CWTSpaceVector<T> &, const T & = CWTZero<T>());
    // creates a quaternion, index runs from left to right
    CWTQuaternion(const T &, const T &, const T &, const T &);
    CWTQuaternion(const T &);
    CWTQuaternion(const CWTMatrix<T> &, unsigned, unsigned);
    // creates a quaternion mapped into a vector
    CWTQuaternion(const CWTVector<T>& vec, unsigned irowStart);

    // constructor from exponential form: q = r*exp(svec*angle), svec
    // should be a unity vector, angle is in radians
    CWTQuaternion(const T &r, const CWTSpaceVector<T> &svec, const T &angle);

    ~CWTQuaternion() {};

    void dimension() { CWTVector<T>::dimension(4); };
    void mapInto(const CWTMatrix<T> &mat,
		 unsigned irowStart,
		 unsigned icolStart);
    void mapInto(const CWTVector<T> &vec, unsigned irowStart);

    CWTQuaternion operator +(const CWTQuaternion &) const;
    CWTQuaternion operator -(const CWTQuaternion &) const;
    CWTQuaternion operator -() const;
    CWTQuaternion operator *(const T &) const;
    CWTQuaternion operator *(const CWTQuaternion &) const;
    CWTQuaternion operator /(const T &value) const
    {
      // NOTE: This used to work with g++ <= 3.2.
      // return (*this)*static_cast<const T &>(CWTUnity<T>()/value);

      T tTmp = CWTUnity<T>();
      tTmp /= value;
      return (*this)*tTmp;
    }
    CWTQuaternion operator /(const CWTQuaternion &) const;

    // not inherited
    CWTQuaternion & operator =(const CWTQuaternion &);
    CWTQuaternion & operator =(const CWTSpaceVector<T> &);
    CWTQuaternion & operator +=(const CWTQuaternion &);
    CWTQuaternion & operator -=(const CWTQuaternion &);
    CWTQuaternion & operator *=(const T &);
    CWTQuaternion & operator *=(const CWTQuaternion &);
    CWTQuaternion & operator /=(const T &);
    CWTQuaternion & operator /=(const CWTQuaternion &);

    // stores product of qtn*qtn in this
    void storeProduct(const CWTQuaternion &, const CWTQuaternion &);
    // stores conjugate of argument in this
    void storeConjugate(const CWTQuaternion &);
    // makes this its own conjugate
    void makeConjugate();

    // returns a unit vector with same direction as this
    CWTQuaternion unit() const { return (*this)/(this->norm()); }
  };

  // Unity quaternion.
  template <class T>
  class CWTUnity< CWTQuaternion<T> >: public CWTQuaternion<T>
  {
  public:
    CWTUnity()
    {
      (*this)[0] = (*this)[1] = (*this)[2] = CWTZero<T>();
      (*this)[3] = CWTUnity<T>();
    }
  };

  // Zero quaternion.
  template <class T>
  class CWTZero< CWTQuaternion<T> >: public CWTQuaternion<T>
  {
  public:
    CWTZero() { fill(CWTZero<T>()); }
  };

  //
  // Constructors
  //
  template < class T >
  inline CWTQuaternion<T>::CWTQuaternion(const T &elemIm0,
					 const T &elemIm1,
					 const T &elemIm2,
					 const T &elemReal)
    :
    CWTVector<T>(4U)
  {
    (*this)[0] = elemIm0;
    (*this)[1] = elemIm1;
    (*this)[2] = elemIm2;
    (*this)[3] = elemReal;
  }

  template < class T >
  inline CWTQuaternion<T>::CWTQuaternion(const T &elemReal)
    :
    CWTVector<T>(4U)
  {
    (*this)[0] = CWTZero<T>();
    (*this)[1] = CWTZero<T>();
    (*this)[2] = CWTZero<T>();
    (*this)[3] = elemReal;
  }

  template < class T >
  inline CWTQuaternion<T>::CWTQuaternion(const CWTSpaceVector<T> &svecIm,
					 const T &elemReal)
    :
    CWTVector<T>(4U)
  {
    (*this)[0] = svecIm[0];
    (*this)[1] = svecIm[1];
    (*this)[2] = svecIm[2];
    (*this)[3] = elemReal;
  }

  template < class T >
  inline CWTQuaternion<T>::CWTQuaternion(const CWTMatrix<T> &mat,
					 unsigned irowStart,
					 unsigned icolStart)
    :
    CWTVector<T>(mat, irowStart, icolStart, irowStart + 3)
  {
  }

  template < class T >
  inline CWTQuaternion<T>::CWTQuaternion(const CWTVector<T>& vec,
					 unsigned irowStart)
    :
    CWTVector<T>(vec, irowStart, irowStart + 3)
  {
  }

  // NOTE: This only works with template arguments that can be
  // converted to floating point types due to the use of sin(3) and
  // cos(3).
  template < class T >
  CWTQuaternion<T>::CWTQuaternion(const T &r,
				  const CWTSpaceVector<T> &svec,
				  const T &angle)
    :
    CWTVector<T>(4U)
  {
    T rsina = r*sin(angle);

    (*this)[0] = svec[0]*rsina;
    (*this)[1] = svec[1]*rsina;
    (*this)[2] = svec[2]*rsina;
    (*this)[3] = r*cos(angle);
  }

  //
  // User Methods
  //

  template < class T >
  inline void CWTQuaternion<T>::mapInto(const CWTMatrix<T> &mat,
					unsigned irowStart,
					unsigned icolStart)
  {
    CWTVector<T>::mapInto(mat, irowStart, icolStart, irowStart + 3);
  }

  template < class T >
  inline void CWTQuaternion<T>::mapInto(const CWTVector<T> &vec,
					unsigned irowStart)
  {
    CWTVector<T>::mapInto(vec, irowStart, irowStart + 3);
  }

  // not inherited
  template < class T >
  inline CWTQuaternion<T> &
  CWTQuaternion<T>::operator =(const CWTQuaternion<T> &qtn)
  {
    return static_cast<CWTQuaternion &>(CWTMatrix<T>::operator=(qtn));
  }

  template < class T >
  inline CWTQuaternion<T> &
  CWTQuaternion<T>::operator =(const CWTSpaceVector<T> &svec)
  {
    (*this)[0] = svec[0];
    (*this)[1] = svec[1];
    (*this)[2] = svec[2];
    (*this)[3] = CWTZero<T>();

    return *this;
  }

  template < class T >
  inline CWTQuaternion<T> &
  CWTQuaternion<T>::operator +=(const CWTQuaternion<T> &qtn)
  {
    return static_cast<CWTQuaternion &>(CWTMatrix<T>::operator+=(qtn));
  }

  template < class T >
  inline CWTQuaternion<T> &
  CWTQuaternion<T>::operator -=(const CWTQuaternion<T> &qtn)
  {
    return static_cast<CWTQuaternion &>(CWTMatrix<T>::operator-=(qtn));
  }

  template < class T >
  inline CWTQuaternion<T> &
  CWTQuaternion<T>::operator *=(const T &value)
  {
    return static_cast<CWTQuaternion &>(CWTMatrix<T>::operator*=(value));
  }

  template < class T >
  inline CWTQuaternion<T> &
  CWTQuaternion<T>::operator *=(const CWTQuaternion<T> &qtn)
  {
    return (*this) = (*this)*qtn;
  }

  template < class T >
  inline CWTQuaternion<T> & CWTQuaternion<T>::operator /=(const T &value)
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this) *= static_cast<const T &>(CWTUnity<T>()/value);

    T tTmp = CWTUnity<T>();
    tTmp /= value;
    return (*this) *= tTmp;
  }

  template < class T >
  inline CWTQuaternion<T> &
  CWTQuaternion<T>::operator /=(const CWTQuaternion<T> &qtn)
  {
    return (*this) = (*this)/qtn;
  }

  template < class T >
  CWTQuaternion<T>
  CWTQuaternion<T>::operator +(const CWTQuaternion<T> &qtn) const
  {
    return CWTQuaternion<T>(*this) += qtn;
  }

  template < class T >
  CWTQuaternion<T>
  CWTQuaternion<T>::operator -(const CWTQuaternion<T> &qtn) const
  {
    return CWTQuaternion<T>(*this) -= qtn;
  }

  template < class T >
  CWTQuaternion<T> CWTQuaternion<T>::operator -() const
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this)*static_cast<const T &>(CWTZero<T>() - CWTUnity<T>());

    T tTmp = CWTZero<T>();
    tTmp -= CWTUnity<T>();
    return (*this)*tTmp;
  }

  template < class T >
  CWTQuaternion<T> CWTQuaternion<T>::operator *(const T &value) const
  {
    return CWTQuaternion<T>(*this) *= value;
  }

  template < class T >
  void CWTQuaternion<T>::storeProduct(const CWTQuaternion<T> &qtnLeft,
				      const CWTQuaternion<T> &qtnRight)
  {
    // to reduce the number of calls to the subscript operator
    T qtnLeft0 = qtnLeft[0];
    T qtnLeft1 = qtnLeft[1];
    T qtnLeft2 = qtnLeft[2];
    T qtnLeft3 = qtnLeft[3];

    T qtnRight0 = qtnRight[0];
    T qtnRight1 = qtnRight[1];
    T qtnRight2 = qtnRight[2];
    T qtnRight3 = qtnRight[3];

    (*this)[0] =
      qtnLeft0*qtnRight3 + qtnLeft1*qtnRight2
      - qtnLeft2*qtnRight1 + qtnLeft3*qtnRight0;

    (*this)[1] =
      -qtnLeft0*qtnRight2 + qtnLeft1*qtnRight3
      + qtnLeft2*qtnRight0 + qtnLeft3*qtnRight1;

    (*this)[2] =
      qtnLeft0*qtnRight1 - qtnLeft1*qtnRight0
      + qtnLeft2*qtnRight3 + qtnLeft3*qtnRight2;

    (*this)[3] =
      -qtnLeft0*qtnRight0 - qtnLeft1*qtnRight1
      - qtnLeft2*qtnRight2 + qtnLeft3*qtnRight3;
  }

  template < class T >
  CWTQuaternion<T>
  CWTQuaternion<T>::operator *(const CWTQuaternion<T> &qtn) const
  {
    CWTQuaternion qtnResult;
    qtnResult.storeProduct(*this, qtn);
    return qtnResult;
  }

  template < class T >
  CWTQuaternion<T>
  CWTQuaternion<T>::operator /(const CWTQuaternion<T> &qtn) const
  {
    return (*this)*inv(qtn);
  }

  // stores conjugate of argument in this
  template < class T >
  void CWTQuaternion<T>::storeConjugate(const CWTQuaternion<T> &qtn)
  {
    // copy argument into this
    (*this) = qtn;
    makeConjugate();
  }

  template < class T >
  void CWTQuaternion<T>::makeConjugate()
  {
    // NOTE: This used to work with g++ <= 3.2.
    // T tmp = static_cast<const T &>(CWTZero<T>() - CWTUnity<T>());

    T tTmp = CWTZero<T>();
    tTmp -= CWTUnity<T>();

    (*this)[0] *= tTmp;
    (*this)[1] *= tTmp;
    (*this)[2] *= tTmp;
  }

  //
  // template functions designed work with the Quaternion class.
  //

  template < class T >
  inline CWTQuaternion<T> operator *(const T &value,
				     const CWTQuaternion<T> &qtn)
  {
    return qtn*value;
  }

  // return real part of a quaternion
  template < class T >
  inline T re(const CWTQuaternion<T> &qtn)
  {
    return qtn[3];
  }

  // returns imaginary (vector) part of a quaternion
  template < class T >
  CWTSpaceVector<T> im(const CWTQuaternion<T> &qtn)
  {
    return CWTSpaceVector<T>(qtn[0], qtn[1], qtn[2]);
  }

  // the conjugate
  template < class T >
  CWTQuaternion<T> conj(const CWTQuaternion<T> &qtn)
  {
    // copy input quaternion
    CWTQuaternion<T> qtnResult(qtn);
    qtnResult.makeConjugate();
    return qtnResult;
  }

  // the inverse
  template < class T >
  CWTQuaternion<T> inv(const CWTQuaternion<T> &qtn)
  {
    T qtn0 = qtn[0];
    T qtn1 = qtn[1];
    T qtn2 = qtn[2];
    T qtn3 = qtn[3];

    // NOTE: This used to work with g++ <= 3.2.
    // return
    //   conj(qtn)
    //     /static_cast<const T &>(qtn0*qtn0 + qtn1*qtn1 + qtn2*qtn2
    //                             + qtn3*qtn3);

    T tTmp = qtn0;
    tTmp *= qtn0;
    tTmp += qtn1*qtn1 + qtn2*qtn2 + qtn3*qtn3;
    return conj(qtn)/tTmp;
  }

  // the sign of a quaternion is a unit quaternion with the same
  // direction
  template <class T>
  inline CWTQuaternion<T>
  sgn(const CWTQuaternion<T> &qtn)
  {
    return qtn.unit();
  }

  // the argument of a quaternion is the angle between it and the
  // scalar number 1 (analogous to the argument of a complex number)
  template <class T>
  inline T
  arg(const CWTQuaternion<T> &qtn)
  {
    return atan2(norm(im(qtn)), re(qtn));
  }

  // quaternion exponentiation
  template <class T>
  CWTQuaternion<T>
  exp(const CWTQuaternion<T> &qtn)
  {
    CWTSpaceVector<T> svec = im(qtn);
    T len = norm(svec);

    if (len == CWTZero<T>())
      {
        return exp(re(qtn))*CWTQuaternion<T>(CWTZero<T>(),
                                             CWTZero<T>(),
                                             CWTZero<T>(),
                                             cos(len));
      }
    else
      {
        return exp(re(qtn))*CWTQuaternion<T>(sgn(svec)*sin(len), cos(len));
      }
  }

  // quaternion logarithm (with base e)
  template <class T>
  CWTQuaternion<T>
  log(const CWTQuaternion<T> &qtn)
  {
    CWTSpaceVector<T> svec = im(qtn);
    T len = norm(svec);

    if (len == CWTZero<T>())
      {
        return CWTQuaternion<T>(CWTZero<T>(),
                                CWTZero<T>(),
                                CWTZero<T>(),
                                log(norm(qtn)));
      }
    else
      {
        return CWTQuaternion<T>(sgn(svec)*arg(qtn), log(norm(qtn)));
      }
  }

  // quaternion power! raise qtn1 to the power qtn2
  template <class T>
  inline CWTQuaternion<T>
  pow(const CWTQuaternion<T> &qtn1, const CWTQuaternion<T> &qtn2)
  {
    return exp(qtn2*log(qtn1));
  }
}

#endif // IG_QUATERN_H
