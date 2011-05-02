/** @file Data.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_KATANA_DATA_H_
#define _GOLEM_DEVICE_KATANA_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Device/Katana/Katana.h>
#include <Golem/Tools/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(KatanaJoint::Desc &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(KatanaArm::Desc &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(KatanaGripper::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_KATANA_DATA_H_*/
