/** @file Wrench.h
 * 
 * Implementation of wrenches.
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_WRENCH_H_
#define _GOLEM_MATH_WRENCH_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Twist.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Wrench representation of a generalised force acting on a rigid body.
*	Wrench is a dual of twist.
*/
typedef Twist Wrench;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_WRENCH_H_*/
