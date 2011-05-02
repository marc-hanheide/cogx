/** @file Types.h
 * 
 * Basic C++ types
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEFS_TYPES_H_
#define _GOLEM_DEFS_TYPES_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/System.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/**	C++ Basic types
*	see: http://www-128.ibm.com/developerworks/library/l-port64.html
*/
#ifdef WIN32
// Windows 32/64-bit, ILP32/LLP64
typedef signed char			I8;
typedef signed short			I16;
typedef signed int			I32;
typedef __int64				I64;

typedef unsigned char			U8;
typedef unsigned short		U16;
typedef unsigned int			U32;
typedef unsigned __int64		U64;

typedef float					F32;
typedef double					F64;

#elif LINUX
// Linux 32/64-bit, ILP32/LP64
typedef signed char			I8;
typedef signed short			I16;
typedef signed int			I32;
typedef long long				I64;

typedef unsigned char			U8;
typedef unsigned short		U16;
typedef unsigned int			U32;
typedef unsigned long long	U64;

typedef float					F32;
typedef double					F64;

#endif

//------------------------------------------------------------------------------

#ifdef WIN32
/**	BOOL - bool conversion
*	see: http://www.codeproject.com
*/
template <class TargetT, class SourceT> struct boolean_converter {
	static TargetT convert(SourceT b) {
		return (TargetT)b;
	}
};
template <> struct boolean_converter<bool, BOOL> {
	static bool convert(BOOL b) {
		return b ? true : false;
	}
};
template <> struct boolean_converter<BOOL, bool> {
	static BOOL convert(bool b) {
		return b ? TRUE : FALSE;
	}
};
template <class TargetT, class SourceT> TargetT boolean_cast(SourceT b) {
	typedef boolean_converter<TargetT, SourceT> converter_t;
	return converter_t::convert(b);
}
#endif

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEFS_TYPES_H_*/
