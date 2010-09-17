// -*-c++-*-
#ifndef IG_SMATRIX_H
#define IG_SMATRIX_H

// $Id: smatrix.h 184 2008-05-10 19:43:17Z hkuiper $

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

#ifndef IG_MATRIX_H
#include "matrix.h"
#endif

namespace CwMtx
{
  // prefix smat
  template < class T = double >
  class CWTSquareMatrix : public CWTMatrix<T>
  {
  public:
    CWTSquareMatrix(): CWTMatrix<T>() {};
    CWTSquareMatrix(unsigned crowInit): CWTMatrix<T>( crowInit, crowInit) {};
    CWTSquareMatrix(const CWTMatrix<T> &mat): CWTMatrix<T>(mat) {};
    CWTSquareMatrix(const CWTSquareMatrix &smat): CWTMatrix<T>(smat) {};
    CWTSquareMatrix(const CWTMatrix<T> &, unsigned, unsigned, unsigned);
    CWTSquareMatrix(const CWTSquareMatrix<T> &, unsigned, unsigned, unsigned);

    ~CWTSquareMatrix() {};

    void dimension(unsigned crowInit)
    {
      CWTMatrix<T>::dimension(crowInit, crowInit);
    }

    void mapInto(const CWTSquareMatrix &, unsigned, unsigned, unsigned);

    CWTSquareMatrix operator +(const CWTSquareMatrix &) const;
    CWTSquareMatrix operator -(const CWTSquareMatrix &) const;
    CWTSquareMatrix operator -() const;
    CWTSquareMatrix operator *(const T &) const;
    CWTSquareMatrix operator *(const CWTSquareMatrix &) const;
    CWTSquareMatrix operator /(const T &value) const
    {
      // NOTE: This used to work with g++ <= 3.2.
      // return (*this)*static_cast<const T &>(CWTUnity<T>()/value);

      T tTmp = CWTUnity<T>();
      tTmp /= value;
      return (*this)*tTmp;
    }
    CWTSquareMatrix operator /(const CWTSquareMatrix &) const;

    // not inherited
    CWTSquareMatrix & operator =(const CWTSquareMatrix &smat);
    CWTSquareMatrix & operator +=(const CWTSquareMatrix &smat);
    CWTSquareMatrix & operator -=(const CWTSquareMatrix &smat);
    CWTSquareMatrix & operator *=(const T &value);
    CWTSquareMatrix & operator *=(const CWTSquareMatrix &);
    CWTSquareMatrix & operator /=(const T &value);
    CWTSquareMatrix & operator /=(const CWTSquareMatrix &);

    // stores the adjoint of argument in this
    void storeAdjoint(const CWTSquareMatrix &);
    // stores the inverse of argument in this
    void storeInverse(const CWTSquareMatrix &);
    // makes this its own adjoint
    void makeAdjoint();
    // makes this its own inverse
    void makeInverse();
    // makes this a unity matrix
    void makeUnity();
  };

  template <class T, unsigned crow>
  class CWTSMat: public T
  {
  public:
    CWTSMat(): T(crow) {}

    T & operator =(const T &smtx) { return T::operator=(smtx); }
  };

  // Unity square matrix
  template <class T, unsigned crow>
  class CWTUnity< CWTSMat<CWTSquareMatrix<T>, crow> >:
    public CWTSMat<CWTSquareMatrix<T>, crow>
  {
  public:
    CWTUnity() { this->makeUnity(); }
  };

  // Zero matrix.
  template <class T, unsigned crow>
  class CWTZero< CWTSMat<CWTSquareMatrix<T>, crow> >:
    public CWTSMat<CWTSquareMatrix<T>, crow>
  {
  public:
    CWTZero() { fill(CWTZero<T>()); }
  };

  //
  // Constructors
  //

  template < class T >
  inline CWTSquareMatrix<T>::CWTSquareMatrix(const CWTMatrix<T> &mat,
					     unsigned irowStart,
					     unsigned icolStart,
					     unsigned irowEnd)
    :
    CWTMatrix<T>(mat,
		 irowStart,
		 icolStart,
		 irowEnd,
		 icolStart + irowEnd - irowStart)
  {
  }

  template < class T >
  inline CWTSquareMatrix<T>::CWTSquareMatrix(const CWTSquareMatrix<T> &smat,
					     unsigned irowStart,
					     unsigned icolStart,
					     unsigned irowEnd)
    :
    CWTMatrix<T>(smat,
		 irowStart,
		 icolStart,
		 irowEnd,
		 icolStart + irowEnd - irowStart)
  {
  }

  //
  // User Methods
  //

  template < class T >
  inline void CWTSquareMatrix<T>::mapInto(const CWTSquareMatrix<T> &smat,
					  unsigned irowStart,
					  unsigned icolStart,
					  unsigned irowEnd)
  {
    CWTMatrix<T>::mapInto(smat,
			  irowStart,
			  icolStart,
			  irowEnd,
			  icolStart + irowEnd - irowStart);
  }

  // not inherited
  template < class T >
  inline CWTSquareMatrix<T> &
  CWTSquareMatrix<T>::operator =(const CWTSquareMatrix<T> &smat)
  {
    return static_cast<CWTSquareMatrix<T> &>(CWTMatrix<T>::operator=(smat));
  }

  template < class T >
  inline CWTSquareMatrix<T> &
  CWTSquareMatrix<T>::operator +=(const CWTSquareMatrix<T> &smat)
  {
    return static_cast<CWTSquareMatrix<T> &>(CWTMatrix<T>::operator+=(smat));
  }

  template < class T >
  inline CWTSquareMatrix<T> &
  CWTSquareMatrix<T>::operator -=(const CWTSquareMatrix &smat)
  {
    return static_cast<CWTSquareMatrix<T> &>(CWTMatrix<T>::operator-=(smat));
  }

  template < class T >
  inline CWTSquareMatrix<T> &
  CWTSquareMatrix<T>::operator *=(const T &value)
  {
    return static_cast<CWTSquareMatrix<T> &>(CWTMatrix<T>::operator*=(value));
  }

  template < class T >
  inline CWTSquareMatrix<T> &
  CWTSquareMatrix<T>::operator *=(const CWTSquareMatrix<T> &smat)
  {
    return (*this) = (*this)*smat;
  }

  template < class T >
  inline CWTSquareMatrix<T> &
  CWTSquareMatrix<T>::operator /=(const T &value)
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this) *= static_cast<const T &>(CWTUnity<T>()/value);

    T tTmp = CWTUnity<T>();
    tTmp /= value;
    return (*this) *= tTmp;
  }

  template < class T >
  inline CWTSquareMatrix<T> &
  CWTSquareMatrix<T>::operator /=(const CWTSquareMatrix<T> &smat)
  {
    return (*this) = (*this)/smat;
  }

  template < class T >
  CWTSquareMatrix<T>
  CWTSquareMatrix<T>::operator +(const CWTSquareMatrix<T> &smat) const
  {
    return CWTSquareMatrix<T>(*this) += smat;
  }

  template < class T >
  CWTSquareMatrix<T>
  CWTSquareMatrix<T>::operator -(const CWTSquareMatrix<T> &smat) const
  {
    return CWTSquareMatrix<T>(*this) -= smat;
  }

  template < class T >
  CWTSquareMatrix<T> CWTSquareMatrix<T>::operator -() const
  {
    // NOTE: This used to work with g++ <= 3.2.
    // return (*this)*static_cast<const T &>(CWTZero<T>() - CWTUnity<T>());

    T tTmp = CWTZero<T>();
    tTmp -= CWTUnity<T>();
    return (*this)*tTmp;
  }

  template < class T >
  CWTSquareMatrix<T> CWTSquareMatrix<T>::operator *(const T &value) const
  {
    return CWTSquareMatrix<T>(*this) *= value;
  }

  template < class T >
  CWTSquareMatrix<T>
  CWTSquareMatrix<T>::operator *(const CWTSquareMatrix<T> &smat) const
  {
    CWTSquareMatrix smatResult(this->getRows());
    smatResult.storeProduct(*this, smat);
    return smatResult;
  }

  template < class T >
  CWTSquareMatrix<T>
  CWTSquareMatrix<T>::operator /(const CWTSquareMatrix<T> &smat) const
  {
    return (*this)*inv(smat);
  }

  // stores inverse of argument in this
  template < class T >
  void CWTSquareMatrix<T>::storeInverse(const CWTSquareMatrix<T> &smat)
  {
    // copy input matrix in this
    (*this) = smat;
    makeInverse();
  }

  // makes this a unity matrix
  template < class T >
  void CWTSquareMatrix<T>::makeUnity()
  {
    unsigned crow(this->getRows()), ccol(this->getCols());

    for (unsigned irow = 0; irow < crow; ++irow)
      {
	for (unsigned icol = 0; icol < ccol; ++icol)
	  {
	    if (irow == icol)
	      {
		(*this)[irow][icol] = CWTUnity<T>();
	      }
	    else
	      {
		(*this)[irow][icol] = CWTZero<T>();
	      }
	  }
      }
  }

  // makes this its own adjoint
  template < class T >
  void CWTSquareMatrix<T>::makeAdjoint()
  {
    // we need a copy of this
    CWTSquareMatrix smatCopy(*this);
    // for easier access to crows
    unsigned crowCopy = smatCopy.getRows();
    T elemTmp;
    // will eventually contain det(smatCopy)
    T elemDet = CWTUnity<T>();

    // make this a unity matrix
    makeUnity();

    // start row reduction
    for (unsigned irow = 0; irow < crowCopy; ++irow)
      {
	// if element [irow][irow] is zero, add a row with non-zero
	// element at column irow
	if (smatCopy[irow][irow] == CWTZero<T>())
	  {
	    for (unsigned irowTmp = irow; irowTmp < crowCopy; ++irowTmp)
	      {
		if (smatCopy[irowTmp][irow] != CWTZero<T>())
		  {
		    // has no effect on adj(smat)
		    smatCopy.addRowToRow(irowTmp, irow);
		    // repeat action on this
		    this->addRowToRow(irowTmp, irow);
		    break;
		  };
	      };
	  };

	elemTmp = smatCopy[irow][irow];
        T tTmp = CWTUnity<T>();
        tTmp /= elemTmp;
	smatCopy.multiplyRow(irow, tTmp);
	// repeat action on this
	multiplyRow(irow, tTmp);
	elemDet *= elemTmp;

	for (unsigned irowToAddTo = 0; irowToAddTo < crowCopy; ++irowToAddTo)
	  {
	    if (irowToAddTo != irow)
	      {
		elemTmp = -smatCopy[irowToAddTo][irow];
		smatCopy.addRowToRow(irow, irowToAddTo, elemTmp);
		// repeat action on this
		addRowToRow(irow, irowToAddTo, elemTmp);
	      };
	  };
      };

    // this now contains its adjoint
    (*this) *= elemDet;
  }

  // stores adjoint of input matrix in this
  template < class T >
  void CWTSquareMatrix<T>::storeAdjoint(const CWTSquareMatrix<T> &smat)
  {
    // copy input matrix in this
    (*this) = smat;
    makeAdjoint();
  }

  // makes this its own inverse
  template < class T >
  void CWTSquareMatrix<T>::makeInverse()
  {
    // we need a copy of this
    CWTSquareMatrix smatCopy(*this);
    // for easier access to crows
    unsigned crowCopy = smatCopy.getRows();
    T elemTmp;

    // make this a unity matrix
    makeUnity();

    // start row reduction
    for (unsigned irow = 0; irow < crowCopy; ++irow)
      {
	// if element [irow][irow] is zero, add a row with non-zero
	// element at column irow
	if (smatCopy[irow][irow] == CWTZero<T>())
	  {
	    for (unsigned irowTmp = irow; irowTmp < crowCopy; ++irowTmp)
	      {
		// has no effect on inv(smat)
		if (smatCopy[irowTmp][irow] != CWTZero<T>())
		  {
		    smatCopy.addRowToRow(irowTmp, irow);
		    // repeat action on this
		    this->addRowToRow(irowTmp, irow);
		    break;
		  };
	      };
	  };

	elemTmp = smatCopy[irow][irow];

        // NOTE: This used to work with g++ <= 3.2.
	// smatCopy.multiplyRow(irow,
	//		     static_cast<const T &>(CWTUnity<T>()/elemTmp));
	// multiplyRow(irow, static_cast<const T &>(CWTUnity<T>()/elemTmp));

        T tTmp = CWTUnity<T>();
        tTmp /= elemTmp;
	smatCopy.multiplyRow(irow, tTmp);
	multiplyRow(irow, tTmp);

	for (unsigned irowToAddTo = 0; irowToAddTo < crowCopy; ++irowToAddTo)
	  {
	    if (irowToAddTo != irow)
	      {
		elemTmp = -smatCopy[irowToAddTo][irow];
		smatCopy.addRowToRow(irow, irowToAddTo, elemTmp);
		// repeat action on this
		addRowToRow(irow, irowToAddTo, elemTmp);
	      };
	  };
      };

    // this now contains its inverse
  }

  //
  // template functions designed work with the Square Matrix class.
  //

  // SCLR*CWTSquareMatrix
  template < class T >
  inline CWTSquareMatrix<T> operator *(const T &value,
				       const CWTSquareMatrix<T> &smat)
  {
    return smat*value;
  }

  template < class T >
  CWTSquareMatrix<T> transpose(const CWTSquareMatrix<T> &smat)
  {
    CWTSquareMatrix<T> smatResult(smat.getRows());
    smatResult.storeTranspose(smat);
    return smatResult;
  }

  // returns the adjoint of a square matrix
  template < class T >
  CWTSquareMatrix<T> adj(const CWTSquareMatrix<T> &smat)
  {
    CWTSquareMatrix<T> smatResult(smat);   // copy input matrix
    smatResult.makeAdjoint();
    return smatResult;
  }

  // calculates the inverse of a square matrix
  template < class T >
  CWTSquareMatrix<T> inv(const CWTSquareMatrix<T> &smat)
  {
    // copy input matrix
    CWTSquareMatrix<T> smatResult(smat);
    smatResult.makeInverse();
    return smatResult;
  }

  // calculates the determinant of a square matrix
  template < class T >
  T det(const CWTSquareMatrix<T> &smat)
  {
    // a copy of the input matrix
    CWTSquareMatrix<T> smatCopy(smat);
    unsigned crowCopy = smatCopy.getRows();

    // start row reduction
    T elemTmp;
    // will eventually contain det(smatCopy)
    T elemDet = CWTUnity<T>();

    for (unsigned irow = 0; irow < crowCopy; ++irow)
      {
	// if element [irow][irow] is zero, add a row with non-zero
	// element at column irow
	if (smatCopy[irow][irow] == CWTZero<T>())
	  {
	    for (unsigned irowTmp = irow; irowTmp < crowCopy; ++irowTmp)
	      {
		// has no effect on inv(smat)
		if (smatCopy[irowTmp][irow] != CWTZero<T>())
		  {
		    smatCopy.addRowToRow(irowTmp, irow);
		    break;
		  };
	      };
	  };

	elemTmp =  smatCopy[irow][irow];
	elemDet *= elemTmp;

	if (elemDet == CWTZero<T>())
	  {
	    // once elemDet is zero it will stay zero
	    return elemDet;
	  }

	smatCopy.multiplyRow(irow, CWTUnity<T>()/elemTmp);

	for (unsigned irowToAddTo = 0; irowToAddTo < crowCopy; ++irowToAddTo)
	  {
	    if (irowToAddTo != irow)
	      {
		smatCopy.addRowToRow(irow,
				     irowToAddTo,
				     -smatCopy[irowToAddTo][irow]);
	      };
	  };
      };

    return elemDet;
  }

  // trace
  template < class T >
  T tr(const CWTSquareMatrix<T> &smat)
  {
    T elemTmp = CWTZero<T>();

    for (unsigned c = 0; c < smat.getCols(); c++)
      {
	elemTmp += smat[c][c];
      }

    return elemTmp;
  }

}

#endif // IG_SMATRIX_H
