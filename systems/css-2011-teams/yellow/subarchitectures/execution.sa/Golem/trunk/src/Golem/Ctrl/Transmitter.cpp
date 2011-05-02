/** @file Transmitter.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Ctrl/Transmitter.h>
#include <Golem/Ctrl/Msg.h>
#include <algorithm>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Transmitter::Transmitter(golem::Arm &arm) :
	arm(arm), context(arm.getContext())
{
}

Transmitter::~Transmitter() {
}

bool Transmitter::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgTransmitterInvalidDesc(Message::LEVEL_CRIT, "Transmitter::create(): Invalid description");

	signal = initSignal = desc.initSignal;
	return true;
}

bool Transmitter::set(const GenConfigspaceState &data) {
	CriticalSectionWrapper csw(cs);
	signal.type = Signal::TYPE_JOINTSPACE;
	signal.gjs = data;
	return true;
}

bool Transmitter::set(const GenWorkspaceState &data) {
	CriticalSectionWrapper csw(cs);
	signal.type = Signal::TYPE_WORKSPACE;
	signal.gws = data;
	return true;
}

bool Transmitter::set(const void *data) {
	return true;
}

bool Transmitter::get(Signal &signal) const {
	CriticalSectionWrapper csw(cs);
	signal = this->signal;
	return true;
}

//------------------------------------------------------------------------------

