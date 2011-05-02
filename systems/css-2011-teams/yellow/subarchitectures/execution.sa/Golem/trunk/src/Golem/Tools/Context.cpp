/** @file Context.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/Context.h>
#include <Golem/Tools/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Context::Context() {
}

Context::~Context() {
	if (parallels != NULL && !parallels->joinThreads(threadTimeOut)) {
	}
}

bool Context::create(const Desc &desc) {
	if (!desc.isValid())
		throw MsgContextInvalidDesc(Message::LEVEL_CRIT, "Context::create(): Invalid description");

	initModule();

	randSeed = desc.randSeed;
	messages = desc.messageStreamDesc->create();

	threadTimeOut = desc.threadTimeOut;
	if (desc.threadParallels > 0) {
		parallels.reset(new Parallels(desc.threadParallels, desc.threadTimeOut));
		parallels->startThreads();
	}

	return true;
}

void Context::initModule() {
	Message::setTimer(&timer);
}

//------------------------------------------------------------------------------
