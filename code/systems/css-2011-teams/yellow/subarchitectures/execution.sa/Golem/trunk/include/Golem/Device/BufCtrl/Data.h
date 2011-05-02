/** @file Data.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_BUFCTRL_DATA_H_
#define _GOLEM_DEVICE_BUFCTRL_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Device/BufCtrl/BufCtrl.h>
#include <Golem/Tools/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(BufCtrlJoint::Desc &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(BufCtrlArm::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_BUFCTRL_DATA_H_*/
