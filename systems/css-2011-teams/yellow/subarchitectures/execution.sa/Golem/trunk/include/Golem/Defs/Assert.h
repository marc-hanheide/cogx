/** @file Assert.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEFS_ASSERT_H_
#define _GOLEM_DEFS_ASSERT_H_

#include <assert.h>

#ifndef ASSERT
#ifdef _DEBUG
	#define ASSERT(x) assert(x);
#else
	#define ASSERT(x)
#endif
#endif

#endif /*_GOLEM_DEFS_ASSERT_H_*/
