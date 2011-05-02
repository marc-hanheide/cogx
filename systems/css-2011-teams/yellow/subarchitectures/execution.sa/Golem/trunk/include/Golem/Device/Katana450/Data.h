/** @file Data.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEVICE_KATANA450_DATA_H_
#define _GOLEM_DEVICE_KATANA450_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Device/Katana450/Katana450.h>
#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
template <> void XMLGetAttribute(const std::string &attr, EControllerType &val, const XMLContext* context);
template <> void XMLSetAttribute(const std::string &attr, const EControllerType &val, XMLContext* context);

/** Reads/writes object from/to a given XML context */
void XMLData(Katana450Joint::Desc &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(Katana450Arm::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEVICE_KATANA450_DATA_H_*/
