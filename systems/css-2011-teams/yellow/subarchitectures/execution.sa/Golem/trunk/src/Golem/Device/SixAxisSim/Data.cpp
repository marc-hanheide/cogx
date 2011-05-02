/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Device/SixAxisSim/Data.h>
#include <Golem/Device/BufCtrlSim/Data.h>
#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(SixAxisSimArm::Desc &val, XMLContext* context, bool create) {
	XMLData((BufCtrlSimArm::Desc &)val, context, create);
	golem::XMLData("L0", val.L0, context->getContextFirst("links"), create);
	golem::XMLData("L1", val.L1, context->getContextFirst("links"), create);
	golem::XMLData("L2", val.L2, context->getContextFirst("links"), create);
	golem::XMLData("L3", val.L3, context->getContextFirst("links"), create);
}

//------------------------------------------------------------------------------
