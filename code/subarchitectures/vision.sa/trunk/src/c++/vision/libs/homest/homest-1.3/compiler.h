/////////////////////////////////////////////////////////////////////////////////
// 
//  Non-linear, robust homography estimation
//  Copyright (C) 2003-08  Manolis Lourakis (lourakis **at** ics.forth.gr)
//  Institute of Computer Science, Foundation for Research & Technology - Hellas
//  Heraklion, Crete, Greece.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
/////////////////////////////////////////////////////////////////////////////////

#ifndef _COMPILER_H
#define _COMPILER_H

/* C compiler specific constants & macros */

/* lapack f77 routines name mangling */
#define F77_FUNC(func)    func ## _

/* note: intel's icc defines both __ICC & __INTEL_COMPILER.
 * Also, some compilers other than gcc define __GNUC__,
 * therefore gcc should be checked last
 */
#ifdef _MSC_VER
#define inline __inline // MSVC
#elif !defined(__ICC) && !defined(__INTEL_COMPILER) && !defined(__GNUC__)
#define inline // other than MSVC, ICC, GCC: define empty
#endif

#endif /* _COMPILER_H */
