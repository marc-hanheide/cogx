/** @file Data.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_DATA_H_
#define _GOLEM_CTRL_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Arm.h>
#include <Golem/Tools/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(GenCoord &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(ConfigspaceCoord &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(ExpCoord &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(Joint::Desc &val, XMLContext* context, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(Arm::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_DATA_H_*/
