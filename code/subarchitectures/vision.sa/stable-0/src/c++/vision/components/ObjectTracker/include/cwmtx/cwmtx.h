// -*-c++-*-
#ifndef IG_CWMTX_H
#define IG_CWMTX_H

// $Id: cwmtx.h 165 2008-01-19 19:53:19Z hkuiper $

// CwMtx matrix and vector math library
// Copyright (C) 1999  Harry Kuiper
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

//---------------------------------------------------------------------------
//
// Master include file for CWMatrix and CWVector classes
//
//---------------------------------------------------------------------------

#ifndef IG_MATRIX_H
#include "matrix.h"
#endif

#ifndef IG_SMATRIX_H
#include "smatrix.h"
#endif

#ifndef IG_VECTOR_H
#include "vector.h"
#endif

#ifndef IG_SVECTOR_H
#include "svector.h"
#endif

#ifndef IG_QUATERN_H
#include "quatern.h"
#endif

#ifndef IG_COORDSYS_H
#include "coordsys.h"
#endif

//
// Define a set of commonly used types
//

namespace CwMtx
{
  typedef CWTMatrix<> CWMatrix;
  typedef CWTVector<> CWVector;
  typedef CWTSpaceVector<> CWSpaceVector;
  typedef CWTSquareMatrix<> CWSquareMatrix;
  typedef CWTQuaternion<> CWQuaternion;
}

#endif   // IG_CWMTX_H
