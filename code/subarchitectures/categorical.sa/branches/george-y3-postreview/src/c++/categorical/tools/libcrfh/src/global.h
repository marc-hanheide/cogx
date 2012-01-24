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

#ifndef _AGLOBAL_H_
#define _AGLOBAL_H_

#include <QtCore/QTextStream>
#include <stdlib.h>


/** Text stream for stdout. */
extern QTextStream aout;

/** Text stream for stderr. */
extern QTextStream aerr;


/** Calls malloc. */
template <class T>
inline T *aMalloc(unsigned int size)
  { return reinterpret_cast<T*>(::malloc(sizeof(T)*size)); }


/** Calls free. */
template <class T>
inline void aFree(T *ptr)
  { ::free(ptr); }


/** Calls realloc. */
template <class T>
inline T *aRealloc(T *ptr, unsigned int size)
  { return reinterpret_cast<T*>(::realloc(ptr, sizeof(T)*size)); }


/** Calls memcpy. */
template <class T>
inline T *aMemCopy(T *dest, const T *src, unsigned int n)
  { return reinterpret_cast<T*>(::memcpy(dest, src, sizeof(T)*n)); }


/** Calls memset. */
template <class T>
inline T *aMemSet(T *dest, int c, unsigned int n)
  { return reinterpret_cast<T*>(::memset(dest, c, sizeof(T)*n)); }


/** Indicates a type of e.g. a matrix or an image etc. */
enum AType
{
  /** Unknown type */
  AT_VOID = 0,

  /** Char */
  AT_CHAR = 1,

  /** Unsigned char */
  AT_UCHAR = 2,

  /** Integer */
  AT_INT = 3,

  /** Double */
  AT_DOUBLE = 4
};


/** Returns minimum of two values. */
template <class T>
inline T aMin(T val1, T val2)
  { if (val1<val2) return val1; return val2; }


/** Returns maximum of two values. */
template <class T>
inline T aMax(T val1, T val2)
  { if (val1<val2) return val2; return val1; }


inline bool aIsWhiteChar(char c)
  { return (c==' ') || (c=='\n') || (c=='\t'); }

inline bool aIsNewLine(char c)
  { return (c=='\n'); }


inline int aRound(double x)
{
   return int(x > 0.0 ? x + 0.5 : x - 0.5);
}



#endif

