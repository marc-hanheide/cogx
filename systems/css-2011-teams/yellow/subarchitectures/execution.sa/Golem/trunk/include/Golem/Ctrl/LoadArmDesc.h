/** @file LoadArmDesc.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_CTRL_LOADARMDESC_H_
#define _GOLEM_CTRL_LOADARMDESC_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Arm.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Library function template */
template <typename _Desc> void loadArmDesc(Context* context, const std::string& path, Arm::Desc::Ptr* pArmDesc) {
	using namespace golem;

	// initialise context in a module
	context->initModule();

	// Create XML parser and load configuration file
	XMLParser::Desc parserDesc;
	XMLParser::Ptr pParser = parserDesc.create();

	// Determine configuration file name
	const std::string cfg = path + ".xml";

	// Load config
	FileReadStream fs(cfg.c_str());
	pParser->load(fs);
	// Find program XML root context
	XMLContext* pXMLContext = pParser->getContextRoot()->getContextFirst("golem arm");
	if (pXMLContext == NULL)
		throw MsgArmInvalidConfig(Message::LEVEL_CRIT, "loadArmDesc(): Invalid configuration file: %s", cfg.c_str());

	// Create description and load xml configuration
	_Desc* pDesc = new _Desc;
	pArmDesc->reset(pDesc);
	XMLData(*pDesc, pXMLContext);
}

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_LOADARMDESC_H_*/
