/** @file Data.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_DATA_H_
#define _GOLEM_DEMO_COMMON_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/XMLData.h>
#include <Golem/Demo/Common/Retina.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes description from/to a given context */
void XMLData(Retina::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_DATA_H_*/
