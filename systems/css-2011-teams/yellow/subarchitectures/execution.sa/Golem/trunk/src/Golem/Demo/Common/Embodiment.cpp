/** @file Embodiment.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Universe.h>
#include <Golem/Demo/Common/Embodiment.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Channel::Channel(Embodiment &embodiment) : Object(embodiment.getScene()), embodiment(embodiment) {
}

Channel::~Channel() {
}

bool Channel::create(const Channel::Desc& desc) {
	Object::create(desc); // throws

	name = desc.name;
	return true;
}

void Channel::release() {
	embodiment.channelList.erase(this);
	Object::release();
}

//------------------------------------------------------------------------------

Embodiment::Embodiment(Scene &scene) : Object(scene) {
}

Embodiment::~Embodiment() {
	release();
}

bool Embodiment::create(const Embodiment::Desc& desc) {
	Object::create(desc); // throws

	name = desc.name;
	return true;
}

void Embodiment::release() {
	while (!channelList.empty())
		releaseChannel(*channelList.back());
}

//------------------------------------------------------------------------------

Channel *Embodiment::createChannel(const Channel::Desc &desc) {
	desc.pEmbodiment = this;
	
	getScene().createObject(desc); // throws

	channelList.push_back(ChannelList::Pair(desc.pChannel.get(), desc.pChannel));
	return desc.pChannel.get();
}

void Embodiment::releaseChannel(Channel &channel) {
	getScene().releaseObject(channel);// calls channel.release()
}

//------------------------------------------------------------------------------
