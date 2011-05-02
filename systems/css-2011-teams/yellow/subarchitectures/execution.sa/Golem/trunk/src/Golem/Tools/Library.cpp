/** @file Device.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/Library.h>
#ifndef WIN32
#include <dlfcn.h>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Handle::Handle(HANDLE handle) : handle(handle) {
}

Handle::~Handle() {
	// unload DLL
	if (handle != NULL) {
#ifdef WIN32
		::FreeLibrary(handle);
#else	// WIN32
		::dlclose(handle);
#endif	// WIN32
		handle = NULL;
	}
}

//------------------------------------------------------------------------------
