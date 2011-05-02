/** @file Library.h
 * 
 * Library tools.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TOOLS_LIBRARY_H_
#define _GOLEM_TOOLS_LIBRARY_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Defs/System.h>

//------------------------------------------------------------------------------

#undef GOLEM_LIBRARY_DECLDIR
#define GOLEM_LIBRARY_DECLDIR

#ifdef GOLEM_LIBRARY_DECLDIR_EXPORT // export DLL
#undef GOLEM_LIBRARY_DECLDIR
#define GOLEM_LIBRARY_DECLDIR __declspec(dllexport)
#endif

#ifdef GOLEM_LIBRARY_DECLDIR_IMPORT // import DLL
#undef GOLEM_LIBRARY_DECLDIR
#define GOLEM_LIBRARY_DECLDIR __declspec(dllimport)
#endif

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Handle */
class Handle {
public:
	typedef shared_ptr<Handle> Ptr;
#ifdef WIN32
	typedef HINSTANCE HANDLE; // Windows
#else	// WIN32
	typedef void* HANDLE;
#endif	// WIN32
	/** The only way to pass handle is to create its wrapper class */
	Handle(HANDLE handle);
	/** Releases library */
	~Handle();

	/** Compares to NULL */
	inline operator bool () const {
		return handle != NULL;
	}
	/** Returns handle */
	inline HANDLE get() const {
		return handle;
	}

private:
	/** handle */
	HANDLE handle;
};

/** Object pointer counter */
template<class _Ptr>
class handle_cnt {
public:
	handle_cnt(_Ptr* ptr = NULL) : ptr(ptr), cnt(1) {
	}
	_Ptr* get() {
		return ptr;
	}
	const _Ptr* get() const {
		return ptr;
	}
	void inc() {
		++cnt;
	}
	void dec() {
		if (ptr != NULL && --cnt == 0) {
			Handle::Ptr pHandle = ptr->pHandle;
			delete ptr;
			ptr = NULL;
		}
	}
	
private:
	_Ptr* ptr;
	size_t cnt;
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_TOOLS_LIBRARY_H_*/
