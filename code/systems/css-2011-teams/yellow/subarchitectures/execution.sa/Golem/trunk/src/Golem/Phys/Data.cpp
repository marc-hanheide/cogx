/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Phys/Data.h>
#include <Golem/Tools/Data.h>
#include <Golem/Tools/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(golem::RGBA &val, XMLContext* context, bool create) {
	ASSERT(context)
	
	U32 rgba[4] = {val._rgba.r, val._rgba.g, val._rgba.b, val._rgba.a};
	XMLData("R", rgba[0], context, create);
	XMLData("G", rgba[1], context, create);
	XMLData("B", rgba[2], context, create);
	XMLData("A", rgba[3], context, create);
	val._rgba.r = (U8)rgba[0];
	val._rgba.g = (U8)rgba[1];
	val._rgba.b = (U8)rgba[2];
	val._rgba.a = (U8)rgba[3];
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::Universe::Desc& desc, XMLContext* context, bool create) {
	ASSERT(context)
	
	XMLData("name", desc.name, context, create);

	XMLData("x", desc.windowX, context->getContextFirst("window"));
	XMLData("y", desc.windowY, context->getContextFirst("window"));
	XMLData("width", desc.windowWidth, context->getContextFirst("window"));
	XMLData("height", desc.windowHeight, context->getContextFirst("window"));
	
	XMLData("fps", desc.renderFPS, context->getContextFirst("physx"));
	XMLData("skin_width", desc.skinWidth, context->getContextFirst("physx"));
	XMLData("sleep_lin_vel_squared", desc.sleepLinVelSquared, context->getContextFirst("physx"));
	XMLData("sleep_ang_vel_squared", desc.sleepAngVelSquared, context->getContextFirst("physx"));
	XMLData("max_angular_velocity", desc.maxAngularVelocity, context->getContextFirst("physx"));
	XMLData("bounce_threshold", desc.bounceThreshold, context->getContextFirst("physx"));
	
	XMLData("frame_drop", desc.recorderFrameDrop, context->getContextFirst("recorder"));
	XMLData("file_name", desc.recorderDesc.fileName, context->getContextFirst("recorder"));
	XMLData("file_ext", desc.recorderDesc.fileExt, context->getContextFirst("recorder"));
	XMLData("leading_zeros", desc.recorderDesc.leadingZeros, context->getContextFirst("recorder"));
	XMLData("encoder_str", desc.recorderDesc.encoderString, context->getContextFirst("recorder"));
	XMLData("buffer_len", desc.recorderDesc.bufferLen, context->getContextFirst("recorder"));
}

void golem::XMLData(golem::Scene::Desc& desc, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData("name", desc.name, context, create);

	XMLData("restitution", desc.restitution, context->getContextFirst("physx"));
	XMLData("static_friction", desc.staticFriction, context->getContextFirst("physx"));
	XMLData("dynamic_friction", desc.dynamicFriction, context->getContextFirst("physx"));
	Vec3 tmp;
	tmp.set(&desc.nxSceneDesc.gravity.x);
	XMLData(tmp, context->getContextFirst("physx gravity"));
	tmp.get(&desc.nxSceneDesc.gravity.x);

	XMLData(desc.viewDirDflt, context->getContextFirst("opengl view_dir"));
	XMLData(desc.viewPointDflt, context->getContextFirst("opengl view_point"));
	XMLData(desc.clearColor, context->getContextFirst("opengl clear_color"));
	XMLData("solid", desc.draw.solid, context->getContextFirst("opengl draw"));
	XMLData("wire", desc.draw.wire, context->getContextFirst("opengl draw"));
	XMLData("shadow", desc.draw.shadow, context->getContextFirst("opengl draw"));
	XMLData("debug", desc.draw.debug, context->getContextFirst("opengl draw"));
}

//------------------------------------------------------------------------------
