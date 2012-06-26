/**
 * @author Thomas Mörwald
 * @date April 2009
 *
 * tools for ObjectTracker
 */

#ifndef TRACKER_TOOLS_H_
#define TRACKER_TOOLS_H_

#include <stdio.h>
#include <math.h>
#include <string>
#include <stdexcept>

#include <Golem/Tools/Message.h>
#include <Golem/Math/Math.h>
#include <Golem/Math/Mat34.h>

#include <Tracker/Tracker.h>
#include <Tracker/EdgeTracker.h>
#include <Tracker/TextureTracker.h>
#include <Tracker/Timer.h>
//#include "myPredictor.h"
#include <TomGine/tgMathlib.h>
#include <Tracker/CDataFile.h>
#include <GLWindow/GLWindow.h>

// using namespace Tracking;
// using namespace std;

namespace smlearning {

struct Parameters{
	int width;
	int height;
};


// *************************************************************************************
// Load INI File
std::string getModelFile(const char* filename);

std::string getModelPath(const char* filename);

int getCamWidth(const char* filename);

int getCamHeight(const char* filename);

bool InputControl(Tracking::Tracker* tracker, blortGLWindow::Event& event);

void GetTrackingParameter( Tracking::Tracker::Parameter& params, const char* ini_file);

inline golem::Mat33 TrackingToGolem( const mat3& m )
{
	golem::Mat33 r;
	r.m11 = m.mat[0];
	r.m21 = m.mat[1];
	r.m31 = m.mat[2];
	r.m12 = m.mat[3];
	r.m22 = m.mat[4];
	r.m32 = m.mat[5];
	r.m13 = m.mat[6];
	r.m23 = m.mat[7];
	r.m33 = m.mat[8];
	return r;
}

inline golem::Vec3 TrackingToGolem( const vec3& v )
{
	return golem::Vec3(v.x, v.y, v.z);
}


inline golem::Mat34 TrackingToGolem( const TomGine::tgPose& p )
{
	mat3 rot;
	vec3 trn;
	p.GetPose(rot,trn);
	golem::Mat34 m(TrackingToGolem(rot), TrackingToGolem(trn));
	return m;
}

inline mat3 GolemToTracking( const golem::Mat33& r )
{
	mat3 m;
	m.mat[0] = (float)r.m11;
	m.mat[1] = (float)r.m21;
	m.mat[2] = (float)r.m31;
	m.mat[3] = (float)r.m12;
	m.mat[4] = (float)r.m22;
	m.mat[5] = (float)r.m32;
	m.mat[6] = (float)r.m13;
	m.mat[7] = (float)r.m23;
	m.mat[8] = (float)r.m33;
	return m;
}

inline vec3 GolemToTracking( const golem::Vec3& v )
{
	return vec3((float)v.v1, (float)v.v2, (float)v.v3);
}

inline TomGine::tgPose GolemToTracking( const golem::Mat34& m)
{
	TomGine::tgPose p;
	mat3 rot = GolemToTracking(m.R);
	vec3 trn = GolemToTracking(m.p);
	p.SetPose(rot,trn);
	return p;
}


} //namespace 

#endif 
