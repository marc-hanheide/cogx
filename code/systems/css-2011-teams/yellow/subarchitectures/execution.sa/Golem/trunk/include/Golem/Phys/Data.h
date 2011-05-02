/** @file Data.h
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_DATA_H_
#define _GOLEM_PHYS_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/XMLParser.h>
#include <Golem/Phys/Renderer.h>
#include <Golem/Phys/Universe.h>
#include <Golem/Phys/Scene.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes RGBA from/to a given context */
void XMLData(RGBA &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Universe setup */
void XMLData(Universe::Desc& desc, XMLContext* context, bool create = false);

/** Scene setup */
void XMLData(Scene::Desc& desc, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_DATA_H_*/
