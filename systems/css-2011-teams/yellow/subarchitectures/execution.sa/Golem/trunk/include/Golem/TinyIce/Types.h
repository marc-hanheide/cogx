/** @file Types.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_TINYICE_TYPES_H_
#define _GOLEM_TINYICE_TYPES_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Bounds.h>
#include <Golem/PhysCtrl/PhysReacPlanner.h>
#include <Golem/Device/Katana/Katana.h>
#include <Golem/TinyIce/TinyIceI.h>

//------------------------------------------------------------------------------

namespace golem {
namespace tinyice {

//------------------------------------------------------------------------------

inline Bounds::Type make(tinyice::ShapeType type) {
	return static_cast<Bounds::Type>(type);
}

inline tinyice::ShapeType make(Bounds::Type type) {
	return static_cast<tinyice::ShapeType>(type);
}

inline golem::RGBA make(const tinyice::RGBA& inp) {
	golem::RGBA out;
	out._rgba.r = (U8)Math::round(float(255.0)*inp.r);
	out._rgba.g = (U8)Math::round(float(255.0)*inp.g);
	out._rgba.b = (U8)Math::round(float(255.0)*inp.b);
	out._rgba.a = (U8)Math::round(float(255.0)*inp.a);
	return out;
}

inline tinyice::RGBA make(const golem::RGBA& inp) {
	tinyice::RGBA out;
	out.r = float(inp._rgba.r)/float(255.0);
	out.g = float(inp._rgba.g)/float(255.0);
	out.b = float(inp._rgba.b)/float(255.0);
	out.a = float(inp._rgba.a)/float(255.0);
	return out;
}

//------------------------------------------------------------------------------

inline tinyice::Vec3 make(const golem::Vec3& v) {
	tinyice::Vec3 iv;
	v.get(&iv.v1);
	return iv;
}

inline golem::Vec3 make(const tinyice::Vec3& iv) {
	return golem::Vec3(iv.v1, iv.v2, iv.v3);
}

inline tinyice::Mat33 make(const golem::Mat33& m) {
	tinyice::Mat33 im;
	m.getRow33(&im.m11);
	return im;
}

inline golem::Mat33 make(const tinyice::Mat33& im) {
	golem::Mat33 m;
	m.setRow33(&im.m11);
	return m;
}

inline tinyice::Mat34 make(const golem::Mat34& m) {
	tinyice::Mat34 im;
	im.R = make(m.R);
	im.p = make(m.p);
	return im;
}

inline golem::Mat34 make(const tinyice::Mat34& im) {
	return golem::Mat34(make(im.R), make(im.p));
}

inline std::vector<golem::Mat34> make(const tinyice::Mat34Seq& inp)  {
	std::vector<golem::Mat34> out;
	for (tinyice::Mat34Seq::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tinyice::Mat34Seq make(const std::vector<golem::Mat34>& inp)  {
	tinyice::Mat34Seq out;
	for (std::vector<golem::Mat34>::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tinyice::Twist make(const golem::Twist& t) {
	tinyice::Twist it;
	it.v = make(t.v);
	it.w = make(t.w);
	return it;
}

inline golem::Twist make(const tinyice::Twist& it) {
	return golem::Twist(make(it.v), make(it.w));
}

inline tinyice::Jacobian make(const golem::Jacobian& inp) {
	tinyice::Jacobian out;	
	for (size_t i = 0; i < golem::CONFIG_SPACE_DIM; ++i)
		out.j.push_back(make(inp[i]));
	return out;
}

inline golem::Jacobian make(const tinyice::Jacobian& inp) {
	golem::Jacobian out;
	tinyice::TwistSeq::const_iterator j = inp.j.begin();
	for (size_t i = 0; i < golem::CONFIG_SPACE_DIM && j != inp.j.end(); ++i)
		out[i] = make(*j++);
	return out;
}

//------------------------------------------------------------------------------

inline golem::ConfigspaceCoord make(const tinyice::ConfigspaceCoord& inp)  {
	golem::ConfigspaceCoord out;
	tinyice::DoubleSeq::const_iterator j = inp.c.begin();
	for (size_t i = 0; i < golem::CONFIG_SPACE_DIM && j != inp.c.end(); ++i)
		out[i] = *j++;
	return out;
}

inline tinyice::ConfigspaceCoord make(const golem::ConfigspaceCoord& inp)  {
	tinyice::ConfigspaceCoord out;
	out.c.insert(out.c.begin(), inp.c, inp.c + golem::CONFIG_SPACE_DIM);
	return out;
}

inline golem::GenConfigspaceState make(const tinyice::GenConfigspaceState& inp)  {
	golem::GenConfigspaceState out;
	out.pos = make(inp.pos);
	out.vel = make(inp.vel);
	std::fill(out.acc.c, out.acc.c + CONFIG_SPACE_DIM, SEC_TM_REAL_ZERO);
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tinyice::GenConfigspaceState make(const golem::GenConfigspaceState& inp)  {
	tinyice::GenConfigspaceState out;
	out.pos = make(inp.pos);
	out.vel = make(inp.vel);
	out.t = (Ice::Double)inp.t;
	return out;
}

inline golem::Planner::Trajectory make(const tinyice::GenConfigspaceStateSeq& inp)  {
	golem::Planner::Trajectory out;
	for (tinyice::GenConfigspaceStateSeq::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tinyice::GenConfigspaceStateSeq make(const golem::Planner::Trajectory& inp)  {
	tinyice::GenConfigspaceStateSeq out;
	for (golem::Planner::Trajectory::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

//------------------------------------------------------------------------------

inline golem::GenWorkspaceState make(const tinyice::GenWorkspaceState& inp) {
	golem::GenWorkspaceState out;
	out.pos = make(inp.pos);
	out.vel = make(inp.vel);
	out.t = (SecTmReal)inp.t;
	return out;
}

inline tinyice::GenWorkspaceState make(const golem::GenWorkspaceState& inp) {
	tinyice::GenWorkspaceState out;
	out.pos = make(inp.pos);
	out.vel = make(inp.vel);
	out.t = (Ice::Double)inp.t;
	return out;
}

//------------------------------------------------------------------------------

inline golem::KatanaGripper::SensorData make(const tinyice::KatanaSensorData& inp) {
	golem::KatanaGripper::SensorData out;
	out.index = inp.index;
	out.value = inp.value;
	return out;
}

inline tinyice::KatanaSensorData make(const golem::KatanaGripper::SensorData& inp) {
	tinyice::KatanaSensorData out;
	out.index = inp.index;
	out.value = inp.value;
	return out;
}

inline golem::KatanaGripper::GripperEncoderData make(const tinyice::KatanaGripperEncoderData& inp) {
	golem::KatanaGripper::GripperEncoderData out;
	out.open = inp.open;
	out.closed = inp.closed;
	out.current = inp.current;
	return out;
}

inline tinyice::KatanaGripperEncoderData make(const golem::KatanaGripper::GripperEncoderData& inp) {
	tinyice::KatanaGripperEncoderData out;
	out.open = inp.open;
	out.closed = inp.closed;
	out.current = inp.current;
	return out;
}

inline golem::KatanaGripper::SensorDataSet make(const tinyice::KatanaSensorDataSet& inp) {
	golem::KatanaGripper::SensorDataSet out;
	for (tinyice::KatanaSensorDataSet::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

inline tinyice::KatanaSensorDataSet make(const golem::KatanaGripper::SensorDataSet& inp) {
	tinyice::KatanaSensorDataSet out;
	for (golem::KatanaGripper::SensorDataSet::const_iterator i = inp.begin(); i != inp.end(); ++i)
		out.push_back(make(*i));
	return out;
}

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_TINYICE_TYPES_H_*/
