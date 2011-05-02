/** @file Types.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TINY_TYPES_H
#define _GOLEM_TINY_TYPES_H

//------------------------------------------------------------------------------

#include <Golem/Tiny/Tiny.h>
#include <Golem/Math/Bounds.h>
#include <Golem/Device/Katana/Katana.h>

//------------------------------------------------------------------------------

namespace golem {
namespace tiny {

//------------------------------------------------------------------------------

inline Bounds::Type make(tiny::ShapeType type) {
	return static_cast<Bounds::Type>(type);
}

inline tiny::ShapeType make(Bounds::Type type) {
	return static_cast<tiny::ShapeType>(type);
}

inline golem::RGBA make(const tiny::RGBA& inp) {
	golem::RGBA out;
	out._rgba.r = (U8)Math::round(float(255.0)*inp.r);
	out._rgba.g = (U8)Math::round(float(255.0)*inp.g);
	out._rgba.b = (U8)Math::round(float(255.0)*inp.b);
	out._rgba.a = (U8)Math::round(float(255.0)*inp.a);
	return out;
}

inline tiny::RGBA make(const golem::RGBA& inp) {
	tiny::RGBA out;
	out.r = float(inp._rgba.r)/float(255.0);
	out.g = float(inp._rgba.g)/float(255.0);
	out.b = float(inp._rgba.b)/float(255.0);
	out.a = float(inp._rgba.a)/float(255.0);
	return out;
}

//------------------------------------------------------------------------------

inline tiny::Jacobian make(const golem::Jacobian& inp) {
	tiny::Jacobian out;
	ASSERT(tiny::CONFIG_SPACE_DIM == golem::CONFIG_SPACE_DIM)
	std::copy(inp.j, inp.j + tiny::CONFIG_SPACE_DIM, out.j);
	return out;
}

inline golem::Jacobian make(const tiny::Jacobian& inp) {
	golem::Jacobian out;
	ASSERT(tiny::CONFIG_SPACE_DIM == golem::CONFIG_SPACE_DIM)
	std::copy(inp.j, inp.j + golem::CONFIG_SPACE_DIM, out.j);
	return out;
}

//------------------------------------------------------------------------------

inline golem::ConfigspaceCoord make(const tiny::ConfigspaceCoord& inp)  {
	golem::ConfigspaceCoord out;
	ASSERT(tiny::CONFIG_SPACE_DIM == golem::CONFIG_SPACE_DIM)
	std::copy(inp.c, inp.c + golem::CONFIG_SPACE_DIM, out.c);
	return out;
}

inline tiny::ConfigspaceCoord make(const golem::ConfigspaceCoord& inp)  {
	tiny::ConfigspaceCoord out;
	ASSERT(tiny::CONFIG_SPACE_DIM == golem::CONFIG_SPACE_DIM)
	std::copy(inp.c, inp.c + tiny::CONFIG_SPACE_DIM, out.c);
	return out;
}

inline golem::GenConfigspaceState make(const tiny::GenConfigspaceState& inp)  {
	golem::GenConfigspaceState out;
	out.pos = make(inp.pos);
	out.vel = make(inp.vel);
	std::fill(out.acc.c, out.acc.c + CONFIG_SPACE_DIM, SEC_TM_REAL_ZERO);
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tiny::GenConfigspaceState make(const golem::GenConfigspaceState& inp)  {
	tiny::GenConfigspaceState out;
	out.pos = make(inp.pos);
	out.vel = make(inp.vel);
	out.t = (double)inp.t;
	return out;
}

inline golem::Planner::Trajectory make(const tiny::GenConfigspaceStateSeq& inp)  {
	golem::Planner::Trajectory out;
	for (tiny::GenConfigspaceStateSeq::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tiny::GenConfigspaceStateSeq make(const golem::Planner::Trajectory& inp)  {
	tiny::GenConfigspaceStateSeq out;
	for (golem::Planner::Trajectory::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

//------------------------------------------------------------------------------

inline golem::GenWorkspaceState make(const tiny::GenWorkspaceState& inp) {
	golem::GenWorkspaceState out;
	out.pos = inp.pos;
	out.vel = inp.vel;
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tiny::GenWorkspaceState make(const golem::GenWorkspaceState& inp) {
	tiny::GenWorkspaceState out;
	out.pos = inp.pos;
	out.vel = inp.vel;
	out.t = (double)inp.t;
	return out;
}

//------------------------------------------------------------------------------

inline golem::KatanaGripper::SensorData make(const tiny::KatanaSensorData& inp) {
	golem::KatanaGripper::SensorData out;
	out.index = inp.index;
	out.value = inp.value;
	return out;
}

inline tiny::KatanaSensorData make(const golem::KatanaGripper::SensorData& inp) {
	tiny::KatanaSensorData out;
	out.index = inp.index;
	out.value = inp.value;
	return out;
}

inline golem::KatanaGripper::GripperEncoderData make(const tiny::KatanaGripperEncoderData& inp) {
	golem::KatanaGripper::GripperEncoderData out;
	out.open = inp.open;
	out.closed = inp.closed;
	out.current = inp.current;
	return out;
}

inline tiny::KatanaGripperEncoderData make(const golem::KatanaGripper::GripperEncoderData& inp) {
	tiny::KatanaGripperEncoderData out;
	out.open = inp.open;
	out.closed = inp.closed;
	out.current = inp.current;
	return out;
}

inline golem::KatanaGripper::SensorDataSet make(const tiny::KatanaSensorDataSet& inp) {
	golem::KatanaGripper::SensorDataSet out;
	for (tiny::KatanaSensorDataSet::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tiny::KatanaSensorDataSet make(const golem::KatanaGripper::SensorDataSet& inp) {
	tiny::KatanaSensorDataSet out;
	for (golem::KatanaGripper::SensorDataSet::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_TINY_TYPES_H*/
