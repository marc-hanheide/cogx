/** @file System.h
 * 
 * Operating system specific headers for Windows and Linux/Cygwin.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEFS_SYSTEM_H_
#define _GOLEM_DEFS_SYSTEM_H_

//------------------------------------------------------------------------------

#if !defined(WIN32) && !defined(LINUX)
#error unknown platform
#endif

#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN
#endif
#ifndef _WIN32_WINNT
	#define _WIN32_WINNT 0x0500
#endif
	#define NOMINMAX

#include <windows.h>

#elif LINUX
#endif

//------------------------------------------------------------------------------

#endif /*_GOLEM_DEFS_SYSTEM_H_*/
