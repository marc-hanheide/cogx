/** @file Pushing.h
 *
 * vHanz02
 *
 * Header file for Pushing.cpp
 * 
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI
 * @author      Sergio Roa - DFKI
 * @version 1.0
 *
 */

#ifndef SMLEARNING_PUSHING_H_
#define SMLEARNING_PUSHING_H_


#include <demo/Common.h>
#include <demo/Tools.h>
#include <demo/Creator.h>
#include <demo/XMLParser.h>
#include <demo/XMLData.h>
#include <demo/Actuator.h>
#include <controller/PhysReacPlanner.h>
#include <controller/Katana.h>
#include <controller/Simulator.h>
#include <iostream>
#include "tools/data_handling.h"


using namespace std;
using namespace golem;
using namespace golem::ctrl;
using namespace golem::phys;
using namespace golem::demo;

#define MAX_PLANNER_TRIALS 50

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

class MyPRMPlanner : public PRMPlanner {

	
protected:	
	/** Planner constructor */
	
	MyPRMPlanner(golem::ctrl::Arm &arm) : PRMPlanner(arm) {}
	
public:	

	virtual bool find(Path &path, Path::iterator iter, const golem::ctrl::GenJointState &begin, const golem::ctrl::GenWorkspaceState &wend) {
		//while (!PRMPlanner::find (path, iter, begin, wend)) {
		for (int i=0; i<MAX_PLANNER_TRIALS; i++) {
			if (PRMPlanner::find (path, iter, begin, wend)) {
				return true;
			}
		
			//cout << "unable to find path... trying again..." << endl;
			context.getLogger()->post(PRMPlannerMsg(StdMsg::LEVEL_INFO, "unable to find path... trying again..."));
		}
		
		return false;
	}

	virtual bool find(Path &path, Path::iterator iter, const golem::ctrl::GenJointState &begin, const golem::ctrl::GenJointState &jend) {
		//while (!PRMPlanner::find (path, iter, begin, jend)) {
		for (int i=0; i<MAX_PLANNER_TRIALS; i++) {
			if (PRMPlanner::find (path, iter, begin, jend)) {
				return true;
			}
			context.getLogger()->post(PRMPlannerMsg(StdMsg::LEVEL_INFO, "unable to find path... trying again..."));
		}

		return false;
	}
};


//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------


//planer setup
template <typename Desc> void setupPlanner(Desc &desc, XMLContext* xmlContext, golem::Context& context);

//creates an object, in this case the polyflap
Actor* setupObjects(Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, golem::Context &context);

//creates a finger actor and sets bounds
void createFinger(std::vector<Bounds::Desc::Ptr> &bounds, const Mat34 &pose, MemoryStream &buffer);

//sets behaviour of joint
NxJoint *setupJoint(NxActor *effector, NxActor *tool, const NxVec3 &anchor, const NxVec3 &axis);

// Modify shape of the joint by adding a new Actor.
void addFinger(PhysReacPlanner &physReacPlanner, U32 jointIndex, std::vector<Bounds::Desc::Ptr> &bounds, golem::Context::Ptr context);

//function for normalizing values according to given bounds (before storing)
Real normalize(const Real& value, const Real& min, const Real& max);	

//function that checks if arm hitted the polyflap while approaching it
bool checkPfPosition(Scene* Scene, const Actor* polyFlapActor, const Vec3& refPos1, const Vec3& refPos2);

void setMovementAngle(const int angle, golem::ctrl::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec);

Vec3 computeOrthogonalVec(const Vec3& normalVec);

Vec3 computeNormalVector(const Vec3& vector1, const Vec3& vector2);

void setPointCoordinates(Vec3& position, const Vec3& normalVec, const Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical);

void setCoordinatesIntoTarget(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top);


void writeDownCollectedData(DataSet data);

#endif /*SMLEARNING_PUSHING_H_*/
