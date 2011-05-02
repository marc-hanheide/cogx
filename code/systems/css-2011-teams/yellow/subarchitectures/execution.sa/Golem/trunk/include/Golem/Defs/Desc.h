/** @file Desc.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEFS_DESC_H_
#define _GOLEM_DEFS_DESC_H_

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Object creating function from the description. */
#define CREATE_FROM_OBJECT_DESC0(OBJECT, POINTER) virtual POINTER create() const {\
	OBJECT *pObject = new OBJECT();\
	POINTER pointer(pObject);\
	if (!pObject->create(*this))\
		pointer.release();\
	return pointer;\
}

/** Object creating function from the description. */
#define CREATE_FROM_OBJECT_DESC1(OBJECT, POINTER, PARAMETER) virtual POINTER create(PARAMETER parameter) const {\
	OBJECT *pObject = new OBJECT(parameter);\
	POINTER pointer(pObject);\
	if (!pObject->create(*this))\
		pointer.release();\
	return pointer;\
}

/** Object creating function from the description. */
#define CREATE_FROM_OBJECT_DESC2(OBJECT, POINTER, PARAMETER1, PARAMETER2) virtual POINTER create(PARAMETER1 parameter1, PARAMETER2 parameter2) const {\
	OBJECT *pObject = new OBJECT(parameter1, parameter2);\
	POINTER pointer(pObject);\
	if (!pObject->create(*this))\
		pointer.release();\
	return pointer;\
}

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEFS_DESC_H_*/
