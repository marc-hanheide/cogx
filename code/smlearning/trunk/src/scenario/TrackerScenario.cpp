/** @file TrackerScenario.h
 *
 * Tracker
 *
 * @author	Marek Kopicki (The University Of Birmingham)
 * @author      Sergio Roa (DFKI)
 *
 * @version 1.0
 *
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.
 */

#include <scenario/TrackerScenario.h>


using namespace TomGine;
using namespace blortGLWindow;
using namespace Tracking;

namespace smlearning {

TrackerScenario::TrackerScenario (golem::Scene &scene) :
	Scenario(scene)
{
}

TrackerScenario::~TrackerScenario ()
{
	// delete tracker_th;
}

void TrackerScenario::init(boost::program_options::variables_map vm)
{
	Scenario::init (vm);
}

bool TrackerScenario::create(const TrackerScenario::Desc& desc) {
	if (!Scenario::create(desc)) // throws
		return false;
	
	this->desc = desc;

	// Tracker
	golem::Mat34 object_initial_pose; 
	fromCartesianPose (object_initial_pose, desc.descActorObject.startPosition, desc.descActorObject.startRotation);

	tracker_th = new TrackerThread (desc.trackerConfig, desc.cameraCalibrationFile, desc.poseCalibrationFile, GolemToTracking (object_initial_pose));

	// _concreteActor = new Polyflap;
	_concreteActor = new Box;
	
	return true;
}

void TrackerScenario::preprocess (golem::SecTmReal elapsedTime)
{
}

void TrackerScenario::postprocess (golem::SecTmReal elapsedTime)
{
	if (_concreteActor == NULL)
		return;
	if (object != NULL)
	{
		CriticalSectionWrapper csw (cs);
		objectPose = TrackingToGolem (tracker_th->getPose ());
		// object->setPose (_concreteActor->getPose ());
		Real obRoll, obPitch, obYaw;
		objectPose.R.toEuler (obRoll, obPitch, obYaw);
		// cout << objectPose.p.v1 << ", " << objectPose.p.v2 << ", " << objectPose.p.v3 << ", " << obRoll << ", " << obPitch << ", " << obYaw << endl;
		if (!isnan(obRoll) && !isnan(obPitch) && !isnan(obYaw))
			object->setPose (objectPose);
		// cout << object->getPose().p.v1 << ", " << object->getMyPose().p.v2 << ", " << object->getMyPose().p.v3 << endl;
	}
}

void TrackerScenario::closeGripper(Katana300Arm &arm) {
	MessageStream* stream = arm.getContext().getMessageStream();
	Katana300Arm::SensorDataSet zero, reading, threshold;

	if (!arm.gripperRecvSensorData(reading))
	{
		cout << "Error receiving data... Have you activated the gripper in GolemDeviceKatana300?" << endl;
		return;
	}
	
	for (size_t i = 0; i < reading.size(); i++)
		stream->write("Sensor #%d: {%d, %d}", i, reading[i].index, reading[i].value);

	threshold = zero = reading;
	for (size_t i = 0; i < threshold.size(); i++)
		threshold[i].value += 50;
	
	stream->write("Closing gripper, low sensitivity...");
	arm.gripperClose(threshold, 5000);
	
	arm.gripperRecvSensorData(reading);
	for (size_t i = 0; i < reading.size(); i++)
		stream->write("Sensor #%d: {%d, %d}", i, reading[i].index, reading[i].value);
	
}

void TrackerScenario::chooseAction ()
{
	arm->setPushDuration (5);
	arm->setHorizontalAngle (chooseAngle(60.0, 120.0));
}

void TrackerScenario::calculateStartCoordinates ()
{
	golem::Vec3 position = object->getPosition ();
	golem::Vec3 normal = object->getNormalVec ();
	golem::Vec3 orthogonal = object->getOrthogonalVec ();
	cout << "orig pos: " << position.v1 << ", " << position.v2 << ", " << position.v3 << endl;
	// positionTarget.set (Real(0.0),Real(0.2),Real(0.05));
	positionTarget.set (position.v1, position.v2, position.v3);
	set_point_coordinates (positionTarget, normal, orthogonal, -desc.descActorObject.dist, 0.0, desc.descActorObject.center);
	cout << "calc pos: " << positionTarget.v1 << ", " << positionTarget.v2 << ", " << positionTarget.v3 << ", " << orientationTarget.v1 << ", " << orientationTarget.v2 << ", " << orientationTarget.v3 << endl;
	fromCartesianPose (target.pos, positionTarget, orientationTarget);

	target.vel.setId(); // it doesn't move
	
	target.t = context.getTimer().elapsed() + arm->getDeltaAsync() + desc.descArmActor.minDuration; // i.e. the movement will last at least 5 sec
}

void TrackerScenario::initMovement()
{
	// Trajectory duration calculated from a speed constant.

	cout << "push duration: " << arm->getPushDuration() << endl;		
	duration 		= arm->getPushDuration();
	// Trajectory end pose equals begin + shift along Y axis
	end 			= target.pos;	

	Vec3 normal 		= object->getNormalVec();
	Vec3 orthogonal 	= object->getOrthogonalVec();
	Vec3 position		= object->getPosition();

	Vec3 centerNormalVec =
		computeNormalVector(
				    Vec3 (positionTarget.v1, positionTarget.v2, positionTarget.v3),
				    Vec3 (position.v1, position.v2, desc.descActorObject.center)
				    );
	//and it's orthogonal
	Vec3 centerOrthogonalVec = computeOrthogonalVec(centerNormalVec);

	setMovementAngle(arm->getHorizontalAngle(), end, desc.distance, centerNormalVec, centerOrthogonalVec);
	cout << "Horizontal direction angle: " << arm->getHorizontalAngle() << " degrees" << endl;

}


void TrackerScenario::run (int argc, char* argv[])
{
	// Create arm
	pKatana300Arm = static_cast<Katana300Arm*>(&(arm->getArm()->getArm()));
	if (pKatana300Arm == NULL)
	throw Message(Message::LEVEL_CRIT, "It is not Katana 300!");
		
	// Display arm information
	armInfo(arm->getArm()->getArm());
	Mat34 armPose = arm->getArm()->getArm().getGlobalPose();
	std::cout << "armpose before: " << armPose.p.v1 << ", " << armPose.p.v2 << ", " << armPose.p.v3 << std::endl;
	golem::Vec3 armoffset (0,0,0.006);
	Mat34 armoffsetPose;
	armoffsetPose.R = armPose.R;
	armoffsetPose.p = armoffset;
	armPose.multiply (armPose, armoffsetPose);
	std::cout << "armpose after: " << armPose.p.v1 << ", " << armPose.p.v2 << ", " << armPose.p.v3 << std::endl;
	arm->getArm()->getArm().setGlobalPose (armPose);
	
	// Close Gripper
	closeGripper(*pKatana300Arm);

	// Wait for some time
	PerfTimer::sleep(SecTmReal(5.0));
	//set: random seed;init arm; get initial config
	std::cout <<"_init "<<std::endl;
	_init();

	// tracker_th->SetThreadType (ThreadTypeIntervalDriven,0);
        tracker_th->Event ();
	// tracker_th->run ();
	// tracker_th->start ();
	creator.setToDefault();
	arm->setCollisionDetection(true);
	object = dynamic_cast<ActorObject*>(scene.createObject(desc.descActorObject));
	object->setShape(scene,_concreteActor,true);
	
	for (unsigned int i=0; i<2; i++)
	{
		// experiment main loops
		std::cout <<"calculate starting coordinates "<<std::endl;
		chooseAction ();
		calculateStartCoordinates ();					
		std::cout << "sending position ";
		cout << Real(target.pos.p.v1) << " " << Real(target.pos.p.v2) << " " << Real(target.pos.p.v3) << endl;
		//move the finger to the beginnnig of experiment trajectory
		arm->sendPosition(context,target , ReacPlanner::ACTION_GLOBAL);
		//compute direction and other features of trajectory
		initMovement ();	
		//move the finger along described experiment trajectory
		arm->moveFinger(context,target, bStart,duration,end);
		// wait for key
		evContinue.wait ();
		evContinue.set(false);

		//move finger to initial position
		arm->moveFingerToStartPose(context);				
	}
	evContinue.set(true);
	arm->moveArmToStartPose(context); 	//move the arm to its initial position
	// tracker_th->Stop ();


}

void TrackerScenario::finish() {
	// m_tracker.release();
	// delete m_tracker;
	// glWindow.release();
	//capture->finish();
	delete tracker_th;
	removeObject();						
}

void TrackerScenario::keyboardHandler(unsigned char key, int x, int y) {
	switch (key) {
	case 'p': case 'P':
		context.getMessageStream()->write(Message::LEVEL_INFO, "PAUSE\n");
		evContinue.set(false);
		break;
	case 'r': case 'R':
		context.getMessageStream()->write(Message::LEVEL_INFO, "RESUME\n");
		evContinue.set(true);
		break;
	}
}

void TrackerScenario::render ()
{
	CriticalSectionWrapper csw (cs);

	if (_concreteActor == NULL)
		return;

	// golem::BoundsRenderer boundsRenderer;
	// boundsRenderer.setMat (_concreteActor->getPose ());
	// boundsRenderer.setWireColour (RGBA::RED);
	// boundsRenderer.renderWire (objectLocalBounds->begin (), objectLocalBounds->end());

}


bool XMLData(TrackerScenario::Desc &val, XMLContext* xmlcontext, Context *context) {
	// XMLData ((Scenario::Desc&)val, xmlcontext, context);
	if (xmlcontext == NULL) {
		ASSERT(false)
		return false;
	}
    
	std::string driver;
	XMLData("driver", driver, xmlcontext->getContextFirst("arm")); // Get arm driver name
	val.descArmActor.armDesc.pArmDesc = Arm::Desc::load(*context, driver); // Load driver
	
	// finger setup
	val.descArmActor.fingerDesc.clear();
	golem::Real baseLength = 0.1848;
	XMLData(baseLength, xmlcontext->getContextFirst("effector base_length"));
	golem::Real fingerLength = 0.135;
	XMLData(fingerLength, xmlcontext->getContextFirst("effector finger_length"));
	golem::Real fingerDiam = 0.02;
	XMLData(fingerDiam, xmlcontext->getContextFirst("effector finger_diameter"));
	golem::Real tipRadius = 0.015;
	XMLData(tipRadius, xmlcontext->getContextFirst("effector tip_radius"));
	
	golem::BoundingBox::Desc* pFingerRodShapeDesc = new golem::BoundingBox::Desc;
	pFingerRodShapeDesc->dimensions.set(fingerDiam/2.0, fingerLength/2.0, fingerDiam/2.0);
	pFingerRodShapeDesc->pose.p.v2 += baseLength + fingerLength/2.0;
	pFingerRodShapeDesc->group = val.descArmActor.effectorGroup;
	val.descArmActor.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerRodShapeDesc));
	/*golem::BoundingSphere::Desc* pFingerTipShapeDesc = new golem::BoundingSphere::Desc;
	pFingerTipShapeDesc->radius = tipRadius;
	pFingerTipShapeDesc->pose.p.v2 += golem::Real(baseLength + fingerLength);
	pFingerTipShapeDesc->group = val.descArmActor.effectorGroup;
	val.descArmActor.fingerDesc.push_back(golem::Bounds::Desc::Ptr(pFingerTipShapeDesc));*/
	
	// end-effector reference pose
	val.descArmActor.referencePose.setId();
	val.descArmActor.referencePose.p.v2 += golem::Real(baseLength + fingerLength);
	
	//polyflap interaction settings

	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	//maximum X value for polyflap position
	XMLData(val.featLimits.maxX, xmlcontext->getContextFirst("ObjectInteraction maxX"));
	//maximum Y value for polyflap position
	XMLData(val.featLimits.maxY, xmlcontext->getContextFirst("ObjectInteraction maxY"));
	//maximum Z value for polyflap position
	XMLData(val.featLimits.maxZ, xmlcontext->getContextFirst("ObjectInteraction maxZ"));
	//minimum X value for polyflap position
	XMLData(val.featLimits.minX, xmlcontext->getContextFirst("ObjectInteraction minX"));
	//minimum Y value for polyflap position
	XMLData(val.featLimits.minY, xmlcontext->getContextFirst("ObjectInteraction minY"));
	//minimum Z value for polyflap position
	XMLData(val.featLimits.minZ, xmlcontext->getContextFirst("ObjectInteraction minZ"));
	//minimum duration value for pushing action
	XMLData(val.featLimits.minDuration, xmlcontext->getContextFirst("ObjectInteraction minPushDuration"));
	//maximum duration value for pushing action
	XMLData(val.featLimits.maxDuration, xmlcontext->getContextFirst("ObjectInteraction maxPushDuration"));
	//minimum value for a label
	XMLData(val.featLimits.minValLabel, xmlcontext->getContextFirst("ObjectInteraction minValLabel"));
	//maximum value for a label
	XMLData(val.featLimits.maxValLabel, xmlcontext->getContextFirst("ObjectInteraction maxValLabel"));

	//minimal duration of a movement (by normal speed)
	XMLData(val.descArmActor.minDuration, xmlcontext->getContextFirst("ObjectInteraction minDuration"));


	Real x;
	Real y;
	Real z;
	
	//Polyflap Position and orientation
	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction startPosition x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction startPosition y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction startPosition z"));
	val.descActorObject.startPosition.set(x, y, z);

	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction startRotation x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction startRotation y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction startRotation z"));
	val.descActorObject.startRotation.set(y, x, z);

	//Polyflap dimensions		
	XMLData(x, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions x"));
	XMLData(y, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions y"));
	XMLData(z, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions z"));
	val.descActorObject.dimensions.set(x, y, z);
	//Polyflap width
	XMLData(val.descActorObject.width, xmlcontext->getContextFirst("ObjectInteraction ObjectDimensions width"));
	

	//vertical distance from the ground
	//const Real over = 0.01;
	//vertical distance from the ground considering fingertip radius
	XMLData(val.descActorObject.over, xmlcontext->getContextFirst("ObjectInteraction over"));
	//distance from the front/back of the polyflap
	XMLData(val.descActorObject.dist, xmlcontext->getContextFirst("ObjectInteraction dist"));

	Real r;

	//distance from the side of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction side"));
	val.descActorObject.side = val.descActorObject.dimensions.v1*r;
	//center of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction center"));
	val.descActorObject.center = val.descActorObject.dimensions.v3*r;
	//distance from the top of the polyflap
	XMLData(r, xmlcontext->getContextFirst("ObjectInteraction top"));
	val.descActorObject.top = val.descActorObject.dimensions.v2 - r;
	//lenght of the movement		
	XMLData(val.distance, xmlcontext->getContextFirst("ObjectInteraction distance"));
	

	XMLData(val.startingPositionsConfig, xmlcontext->getContextFirst("loop startingPositions"));



	
	golem::XMLData("camera_file", val.cameraCalibrationFile, xmlcontext->getContextFirst("camera calibration"));
	golem::XMLData("pose_file", val.poseCalibrationFile, xmlcontext->getContextFirst("camera calibration"));
	golem::XMLData("config_file", val.trackerConfig, xmlcontext->getContextFirst("tracker"));

	return true;

}

void TrackerScenarioApp::run(int argc, char *argv[]) {
	TrackerScenario::Desc desc;
	XMLData(desc, xmlcontext(), context());

	TrackerScenario *pTrackerScenario = dynamic_cast<TrackerScenario*>(scene()->createObject(desc)); // throws
	if (pTrackerScenario == NULL)
		throw Message(Message::LEVEL_CRIT, "TrackerScenarioApp::run(): unable to cast to TrackerScenario");

	// Random number generator seed
	context()->getMessageStream()->write(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);
	
	try {
		// todo 
		//pTrackerScenario->main();
		pTrackerScenario->run(argc, argv);
	}
	catch (const Scenario::Interrupted&) {
		// todo
		pTrackerScenario->finish();
	}

	scene()->releaseObject(*pTrackerScenario);
}




} // namespace
