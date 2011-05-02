/** @file XMLData.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

template <> void golem::XMLGetValue(std::string &val, const XMLContext* context) {
	ASSERT(context)
	val = context->getValue();
}

template <> void golem::XMLSetValue(const std::string &val, XMLContext* context) {
	ASSERT(context)
	context->setValue(val);
}

//------------------------------------------------------------------------------

template <> void golem::XMLGetAttribute(const std::string &attr, std::string &val, const XMLContext* context) {
	ASSERT(context)
	val = context->getAttribute(attr);
}

template <> void golem::XMLSetAttribute(const std::string &attr, const std::string &val, XMLContext* context) {
	ASSERT(context)
	context->setAttribute(attr, val);
}

//------------------------------------------------------------------------------
