/** @file Msg.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_DEMO_COMMON_MSG_H_
#define _GOLEM_DEMO_COMMON_MSG_H_

//------------------------------------------------------------------------------

#include <Golem/PhysCtrl/Msg.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class MsgEmbod : public MsgObject {};
class MsgChannel : public MsgObject {};

class MsgSensor : public MsgChannel {};
class MsgEffector : public MsgChannel {};
class MsgFilter : public MsgChannel {};
class MsgLearner : public MsgChannel {};

class MsgWrenchTransmitter : public MsgTransmitter {MESSAGE_BODY(MsgWrenchTransmitter)};

class MsgFTSensor : public MsgSensor {};

class MsgPointTracker : public MsgSensor {};

class MsgRigidBodyTracker : public MsgSensor {};

class MsgRetina : public MsgFilter {MESSAGE_BODY(MsgRetina)};

class MsgRecognition : public MsgFilter {};

class MsgFinger : public MsgEffector {MESSAGE_BODY(MsgFinger)};

class MsgRotations : public MsgEmbod {MESSAGE_BODY(MsgRotations)};

class MsgImageFilter : public MsgEmbod {MESSAGE_BODY(MsgImageFilter)};

class MsgLayer : public MsgEmbod {MESSAGE_BODY(MsgLayer)};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEMO_COMMON_MSG_H_*/
