/** @file Pointers.h
 * 
 * Implements smart pointers with reference counting.
 * TODO use BOOST implementation instead
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEFS_POINTERS_H_
#define _GOLEM_DEFS_POINTERS_H_

#include <stddef.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Object pointer counter */
template<class _Ptr>
class obj_cnt {
public:
	obj_cnt(_Ptr* ptr = NULL) : ptr(ptr), cnt(1) {
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
			delete ptr;
			ptr = NULL;
		}
	}
	
private:
	_Ptr* ptr;
	size_t cnt;
};

/** Array pointer counter */
template<class _Ptr>
class arr_cnt {
public:
	arr_cnt(_Ptr* ptr = NULL) : ptr(ptr), cnt(1) {
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
			delete [] ptr;
			ptr = NULL;
		}
	}
	
private:
	_Ptr* ptr;
	size_t cnt;
};

//------------------------------------------------------------------------------

/** Object pointer wrapper with reference counting.
 * 
 */
template <typename _Ptr, typename _Cnt = obj_cnt<_Ptr> >
class shared_ptr {
public:
	typedef _Ptr pointer_type;
	typedef _Cnt counter_type;

	/** Explicitly constructs from object pointer.
	 * 
	 * Constructor cannot be used for implicit object construction.
	 * 
	 * @param ptr	object pointer
	 */
	explicit shared_ptr(_Ptr *ptr = NULL) : pCounter(NULL) {
		create(ptr);
	}

	/** Constructs assuming pointer from ref shared_ptr
	 * 
	 * @param ref	object reference
	 */
	shared_ptr(const shared_ptr<_Ptr, _Cnt>& ref) {
		acquire(ref.pCounter);
	}

	/** Destroys the object.
	 */
	~shared_ptr() {
		release();
	}

	/** Assigns ref.
	 */
	shared_ptr<_Ptr, _Cnt>& operator = (const shared_ptr<_Ptr, _Cnt>& ref) {
		if (this != &ref) {
			release();
			acquire(ref.pCounter);
		}
		return *this;
	}

	/** Returns array object reference.
	 */
	_Ptr& operator [] (size_t index) const {
		return pCounter->get()[index];
	}

	/** Returns designated value.
	 */
	_Ptr &operator * () const {
		return *pCounter->get();
	}

	/** Returns pointer to class object.
	 */
	_Ptr *operator -> () const {
		return pCounter->get();
	}
	
	/** Returns wrapped pointer.
	 */
	_Ptr* get() const {
		return pCounter != NULL ? pCounter->get() : NULL;
	}

	/** Releases designated object and store a new pointer.
	 */
	void reset(_Ptr* ptr = NULL) {
		release();
		create(ptr);
	}

	/** Releases and decrement pointer counter.
	 */
	void release() {
		if (pCounter != NULL) {
			pCounter->dec();
			if (pCounter->get() == NULL)
				delete pCounter;
			pCounter = NULL;
		}
	}

	/** Performs dynamic_cast pointer conversion.
	 * 
	 * @param r	converted pointer
	 */
	template <typename _Left, typename _Right> friend _Left dynamic_pointer_cast(const _Right& r);

	/** Performs static_cast pointer conversion.
	 * 
	 * @param r	converted pointer
	 */
	template <typename _Left, typename _Right> friend _Left static_pointer_cast(const _Right& r);

	/** Performs const_cast pointer conversion.
	 * 
	 * @param r	converted pointer
	 */
	template <typename _Left, typename _Right> friend _Left const_pointer_cast(const _Right& r);

private:
	/** Object pointer */
	_Cnt* pCounter;
	
	/** Create a new pointer counter.
	 * 
	 * @param ptr	object pointer
	 */
	void create(_Ptr* ptr) {
		if (ptr != NULL)
			pCounter = new _Cnt(ptr);
	}
	
	/** Acquire and increment a new pointer counter.
	 * 
	 * @param pCounter	acquired pointer counter
	 */
	void acquire(_Cnt* pCounter) {
		this->pCounter = pCounter;
		if (pCounter != NULL)
			pCounter->inc();
	}
};

template <typename _Left, typename _Right> _Left dynamic_pointer_cast(const _Right& r) {
	_Left ptr;

	if (dynamic_cast<typename _Left::pointer_type*>(r.get()) != NULL) {
		ptr.pCounter = (typename _Left::counter_type*)r.pCounter;
		ptr.pCounter->inc();
	}

	return ptr;
}

template <typename _Left, typename _Right> _Left static_pointer_cast(const _Right& r) {
	_Left ptr;

	if (static_cast<typename _Left::pointer_type*>(r.get()) != NULL) {
		ptr.pCounter = (typename _Left::counter_type*)r.pCounter;
		ptr.pCounter->inc();
	}

	return ptr;
}

template <typename _Left, typename _Right> _Left const_pointer_cast(const _Right& r) {
	_Left ptr;

	if (const_cast<typename _Left::pointer_type*>(r.get()) != NULL) {
		ptr.pCounter = (typename _Left::counter_type*)r.pCounter;
		ptr.pCounter->inc();
	}

	return ptr;
}

template <typename _Left, typename _LeftCnt, typename _Right, typename _RightCnt> bool operator == (const shared_ptr<_Left, _LeftCnt>& l, const shared_ptr<_Right, _RightCnt>& r) {
	return l.get() == r.get();
}

template <typename _Ptr, typename _Cnt> bool operator == (const shared_ptr<_Ptr, _Cnt>& l, const void* r) {
	return l.get() == r;
}

template <typename _Ptr, typename _Cnt> bool operator == (const void* l, const shared_ptr<_Ptr, _Cnt>& r) {
	return l == r.get();
}

template <typename _Left, typename _LeftCnt, typename _Right, typename _RightCnt> bool operator != (const shared_ptr<_Left, _LeftCnt>& l, const shared_ptr<_Right, _RightCnt>& r) {
	return l.get() != r.get();
}

template <typename _Ptr, typename _Cnt> bool operator != (const shared_ptr<_Ptr, _Cnt>& l, const void* r) {
	return l.get() != r;
}

template <typename _Ptr, typename _Cnt> bool operator != (const void* l, const shared_ptr<_Ptr, _Cnt>& r) {
	return l != r.get();
}

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEFS_POINTERS_H_*/
