/** @file Msg.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_PHYS_MSG_H_
#define _GOLEM_PHYS_MSG_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Message.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class MsgApplication : virtual public Message {MESSAGE_BODY(MsgApplication)};

class MsgObject : public Message {};
class MsgObjectInvalidDesc : public MsgObject {MESSAGE_BODY(MsgObjectInvalidDesc)};

class MsgActor : public MsgObject {};
class MsgActorNxActorCreate : public MsgActor {MESSAGE_BODY(MsgActorNxActorCreate)};
class MsgActorNxShapeCreate : public MsgActor {MESSAGE_BODY(MsgActorNxShapeCreate)};
class MsgActorNxShapeDescCreate : public MsgActor {MESSAGE_BODY(MsgActorNxShapeDescCreate)};
class MsgActorBoundsDescCreate : public MsgActor {MESSAGE_BODY(MsgActorBoundsDescCreate)};
class MsgActorBoundsCreate : public MsgActor {MESSAGE_BODY(MsgActorBoundsCreate)};
class MsgActorBoundsNxShapeMismatch : public MsgActor {MESSAGE_BODY(MsgActorBoundsNxShapeMismatch)};

class MsgScene : public Message {};
class MsgSceneInvalidDesc : public MsgScene {MESSAGE_BODY(MsgSceneInvalidDesc)};
class MsgSceneNxSceneCreate : public MsgScene {MESSAGE_BODY(MsgSceneNxSceneCreate)};
class MsgSceneDebugInit : public MsgScene {MESSAGE_BODY(MsgSceneDebugInit)};
class MsgSceneObjectCreate : public MsgScene {MESSAGE_BODY(MsgSceneObjectCreate)};
class MsgSceneBoundsDescInvalidDesc : public MsgScene {MESSAGE_BODY(MsgSceneBoundsDescInvalidDesc)};
class MsgSceneBoundsDescCreate : public MsgScene {MESSAGE_BODY(MsgSceneBoundsDescCreate)};
class MsgSceneNxShapeDescInvalidDesc : public MsgScene {MESSAGE_BODY(MsgSceneNxShapeDescInvalidDesc)};
class MsgSceneNxShapeDescCreate : public MsgScene {MESSAGE_BODY(MsgSceneNxShapeDescCreate)};

class MsgUniverse : public Message {};
class MsgUniverseMultipleInstances: public MsgUniverse {MESSAGE_BODY(MsgUniverseMultipleInstances)};
class MsgUniverseInvalidDesc : public MsgUniverse {MESSAGE_BODY(MsgUniverseInvalidDesc)};
class MsgUniversePhysXInit : public MsgUniverse {MESSAGE_BODY(MsgUniversePhysXInit)};
class MsgUniversePhysXCookInit : public MsgUniverse {MESSAGE_BODY(MsgUniversePhysXCookInit)};
class MsgUniverseNoScenes : public MsgUniverse {MESSAGE_BODY(MsgUniverseNoScenes)};
class MsgUniverseThreadLaunch : public MsgUniverse {MESSAGE_BODY(MsgUniverseThreadLaunch)};
class MsgUniverseGlutInit : public MsgUniverse {MESSAGE_BODY(MsgUniverseGlutInit)};
class MsgUniverseSceneCreate : public MsgUniverse {MESSAGE_BODY(MsgUniverseSceneCreate)};

class MsgRecorder : public Message {};
class MsgRecorderInvalidDesc : public MsgRecorder {MESSAGE_BODY(MsgRecorderInvalidDesc)};
class MsgRecorderThreadLaunch : public MsgRecorder {MESSAGE_BODY(MsgRecorderThreadLaunch)};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PHYS_MSG_H_*/
