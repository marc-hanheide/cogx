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


#include "Common.h"
#include "Tools.h"
#include "Creator.h"
#include "controller/PhysReacPlanner.h"
#include "controller/Katana.h"
#include "controller/Simulator.h"
#include <iostream>
#include "XMLParser.h"
#include "XMLData.h"
#include "Actuator.h"
#include "tools/data_handling.h"


using namespace std;
using namespace msk;
using namespace msk::ctrl;
using namespace msk::phys;
using namespace msk::demo;

#define MAX_PLANNER_TRIALS 50

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

class MyPRMPlanner : public PRMPlanner {

	
protected:	
	/** Planner constructor */
	
	MyPRMPlanner(msk::ctrl::Arm &arm) : PRMPlanner::PRMPlanner(arm) {}
	
public:	

	virtual bool find(Path &path, Path::iterator iter, const msk::ctrl::GenJointState &begin, const msk::ctrl::GenWorkspaceState &wend) {
		//while (!PRMPlanner::find (path, iter, begin, wend)) {
		for (int i=0; i<MAX_PLANNER_TRIALS; i++) {
			if (PRMPlanner::find (path, iter, begin, wend)) {
				break;
			}
		
			//cout << "unable to find path... trying again..." << endl;
			context.getLogger()->post(PRMPlannerMsg(StdMsg::LEVEL_INFO, "unable to find path... trying again..."));
		}
		
	}

	virtual bool find(Path &path, Path::iterator iter, const msk::ctrl::GenJointState &begin, const msk::ctrl::GenJointState &jend) {
		//while (!PRMPlanner::find (path, iter, begin, jend)) {
		for (int i=0; i<MAX_PLANNER_TRIALS; i++) {
			if (PRMPlanner::find (path, iter, begin, jend)) {
				break;
			}
			context.getLogger()->post(PRMPlannerMsg(StdMsg::LEVEL_INFO, "unable to find path... trying again..."));
		}
	}
};


//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------


//planer setup
template <typename Desc> void setupPlanner(Desc &desc, XMLContext* xmlContext, msk::Context& context);

//creates an object, in this case the polyflap
Actor* setupObjects(Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, msk::Context &context);

//creates a finger actor and sets bounds
void createFinger(std::vector<Bounds::Desc::Ptr> &bounds, const Mat34 &pose, MemoryStream &buffer);

//sets behaviour of joint
NxJoint *setupJoint(NxActor *effector, NxActor *tool, const NxVec3 &anchor, const NxVec3 &axis);

// Modify shape of the joint by adding a new Actor.
void addFinger(PhysReacPlanner &physReacPlanner, U32 jointIndex, std::vector<Bounds::Desc::Ptr> &bounds, msk::Context::Ptr context);

//function for normalizing values according to given bounds (before storing)
Real normalize(const Real& value, const Real& min, const Real& max);	

//function that checks if arm hitted the polyflap while approaching it
bool checkPfPosition(const Actor* polyFlapActor, const Vec3& refPos1, const Vec3& refPos2);

void setMovementAngle(const int angle, msk::ctrl::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec);

Vec3 computeOrthogonalVec(const Vec3& normalVec);

Vec3 computeNormalVector(const Vec3& vector1, const Vec3& vector2);

void setPointCoordinates(Vec3& position, const Vec3& normalVec, const Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical);

void setCoordinatesIntoTarget(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top);


void writeDownCollectedData(DataSet data);

#endif /*SMLEARNING_PUSHING_H_*/
