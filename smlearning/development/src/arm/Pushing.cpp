/** @file Pushing.cpp
 *
 * vHanz02
 *
 * Program which moves the arm along a straight line
 * using reactive trajectory planner and collision detection
 * simulating a pushing action on a polyflap
 * (also see DemoReacPlannerPhys and DemoRobotFinger).
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 * 
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI
 * @author      Sergio Roa - DFKI
 * @version 1.0
 *
 */

#include "arm/Pushing.h"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

//planer setup
template <typename Desc> void setupPlanner(Desc &desc, XMLContext* xmlContext, msk::Context& context) {
	// some planner parameter tuning
	desc.plannerDesc.pHeuristicDesc->distJointcoordMax.j[4] = Real(1.0*MATH_PI);// last joint
	// Enable signal synchronization (default value)
	//desc.reacPlannerDesc.signalSync = true;
}



//creates an object, in this case the polyflap
Actor* setupObjects(Scene &scene, Vec3 position, Vec3 rotation, Vec3 dimensions, msk::Context &context) {

	//set physical parameters of simulation
	NxMaterial* defaultMaterial = scene.getNxScene()->getMaterialFromIndex(0);
	defaultMaterial->setRestitution((NxReal)0.05);
	defaultMaterial->setStaticFriction((NxReal)0.9);
	defaultMaterial->setDynamicFriction((NxReal)0.5);
					
	// Creator
	Creator creator(scene);
	Actor::Desc *pActorDesc;
	
	Actor *polyFlapActor;
	
	// Create ground plane.
	pActorDesc = creator.createGroundPlaneDesc();
	scene.createObject(*pActorDesc);
	
	// Create polyflap
	pActorDesc = creator.createSimple2FlapDesc(Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(dimensions.v1*0.5), Real(0.0002), REAL_PI_2);
	//-sets coordinates
	pActorDesc->nxActorDesc.globalPose.t.set(NxReal(position.v1), NxReal(position.v2), NxReal(position.v3));	

	Mat34 pose;
	pose.R.fromEuler(
		rotation.v1, 
		rotation.v2, 
		rotation.v3 
	);


	//-sets rotations	
	pActorDesc->nxActorDesc.globalPose.M.setRowMajor(&pose.R._m._11);	
	
	//-density
	pActorDesc->nxActorDesc.density = NxReal(5.0);	

	polyFlapActor = dynamic_cast<Actor*>(scene.createObject(*pActorDesc));

	return polyFlapActor;
	
}


// //methods needed to create and set finger
// template <typename Desc>
// void setupFingerActuator(Desc &desc) {
	

// 	if (xmlContext == NULL)	{
// 		context.getLogger()->post(StdMsg(StdMsg::LEVEL_CRIT, "NULL Actuator context"));
// 		return;
// 	}
	
// 	XMLData(desc.fingerDesc.fingerCtrlDesc.reacPlannerDesc.pTransmitterDesc->initSignal.gws.pos.p, xmlContext->getContextFirst("finger init_pos"));

	
// 	Mat33 initTrnR;
// 	Vec3 tmp;
// 	desc.fingerDesc.fingerCtrlDesc.reacPlannerDesc.pTransmitterDesc->initSignal.gws.pos.q.toMat33(initTrnR);
// 	initTrnR.toEuler(tmp.v1, tmp.v2, tmp.v3); // Y, X, Z
// 	//XMLData(tmp, xmlContext->getContextFirst("finger init_rot_euler"));
// 	initTrnR.fromEuler(tmp.v1, tmp.v2, tmp.v3); // Y, X, Z
// 	desc.fingerDesc.fingerCtrlDesc.reacPlannerDesc.pTransmitterDesc->initSignal.gws.pos.q.fromMat33(initTrnR);




// 	//XMLData(desc.fingerDesc.fingerActorDesc.appearance.solidColour, xmlContext->getContextFirst("finger colour"));
// }

// void setupActuator(Actuator::Desc::Ptr& pDesc) {
// 	//Actuator pActuator = new Actuator
// 	typedef FingerActuator::Desc<GenSimArm, MyPRMPlanner, ReacPlanner> FingerActuatorDesc;
// //  	typedef FingerActuator::Desc<GenSimArm, PRMPlanner, ReacPlanner> FingerActuatorDesc;
// 	FingerActuatorDesc *pFingerActuatorDesc = new FingerActuatorDesc();
// 	pDesc.reset(pFingerActuatorDesc);
// //	pDesc.appearance.solidColour = phys::RGBA::RED;
// 	setupFingerActuator(*pFingerActuatorDesc);
// 	//pActuatorDesc.release();

// }

void trn(Vec3 v [], U32 n, const Mat34 &pose) {
	while (n--)
		pose.multiply(v[n], v[n]);
}

void createFinger(std::vector<Bounds::Desc::Ptr> &bounds, const Mat34 &pose, MemoryStream &buffer) {
	// mace characteristic dimensions
	const Real length = Real(0.1);
	const Real begin = Real(0.01);
	const Real end = Real(0.01);
// 	const Real diam = Real(0.02);
// 	const Real height = Real(0.05);
	const Real pos = length - end;
	
	// objects data
	Vec3 hilt[8];
	BoundingConvexMesh::Desc desc;
	desc.verticesStrideBytes = sizeof(Vec3);
	desc.bCook = true;

	// create hilt (along Y-axis)
	hilt[0].set(-begin, Real(0.0), -begin);
	hilt[1].set(+begin, Real(0.0), -begin);
	hilt[2].set(-begin, Real(0.0), +begin);
	hilt[3].set(+begin, Real(0.0), +begin);
	hilt[4].set(-end, length, -end);
	hilt[5].set(+end, length, -end);
	hilt[6].set(-end, length, +end);
	hilt[7].set(+end, length, +end);
	trn(hilt, 8, pose);
	desc.numOfVertices = 8;
	buffer.write(hilt, 8);
	desc.vertices = buffer.get().back();
	bounds.push_back(desc.clone());

}



NxJoint *setupJoint(NxActor *effector, NxActor *tool, const NxVec3 &anchor, const NxVec3 &axis) {
	const NxReal angle = NxReal(Math::degToRad(0.001));
	const NxReal spring = NxReal(0.01);
	const NxReal damping = NxReal(0.0001);
	
// 	tool->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
	
	NxD6JointDesc jDesc;
	jDesc.actor[0] = effector;
	jDesc.actor[1] = tool;
	jDesc.setGlobalAnchor(anchor);
	jDesc.setGlobalAxis(axis);

	jDesc.xMotion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.yMotion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.zMotion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.linearLimit.value = NxReal(0.0);//3locked/2locked/1locked/0locked = none/line/cylinder/sphere

	jDesc.swing1Motion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.swing1Limit.value = angle;
	jDesc.swing1Limit.spring = spring;
	jDesc.swing1Limit.damping = damping;
	
	jDesc.swing2Motion = NX_D6JOINT_MOTION_LOCKED;
	jDesc.swing2Limit.value = angle;
	jDesc.swing2Limit.spring = spring;
	jDesc.swing2Limit.damping = damping;
	
	jDesc.twistMotion = NX_D6JOINT_MOTION_LOCKED;//NX_D6JOINT_MOTION_LIMITED;
	jDesc.twistLimit.low.value =  -angle;
	jDesc.twistLimit.low.spring = spring;
	jDesc.twistLimit.low.damping = damping;
	jDesc.twistLimit.high.value = angle;
	jDesc.twistLimit.high.spring = spring;
	jDesc.twistLimit.high.damping = damping;

	return effector->getScene().createJoint(jDesc);
}

// Modify shape of the joint by adding a new Actor.
void addFinger(PhysReacPlanner &physReacPlanner, U32 jointIndex, std::vector<Bounds::Desc::Ptr> &bounds, msk::Context::Ptr context) {
	Actor::Desc fingerActorDesc;
	NxBodyDesc nxBodyDesc;
	Arm &arm = physReacPlanner.getArm();


	fingerActorDesc.nxActorDesc.body = &nxBodyDesc;
	fingerActorDesc.nxActorDesc.density = NxReal(10.0);

	for (std::vector<Bounds::Desc::Ptr>::const_iterator i = bounds.begin(); i != bounds.end(); i++) {
		NxShapeDesc *pNxShapeDesc = physReacPlanner.getScene().createNxShapeDesc(*i);
		pNxShapeDesc->density = NxReal(10.0);
		fingerActorDesc.nxActorDesc.shapes.push_back(pNxShapeDesc);
	}

	// Find the current pose of the end-effector
	GenJointState j;
	arm.lookupInp(j, physReacPlanner.getContext().getTimer()->elapsed());
	//arm.lookupInp (j, SEC_TM_REAL_ZERO);
	Mat34 zeroPose;
	arm.forwardTransform(zeroPose, j.pos);

	zeroPose.multiply(zeroPose, arm.getReferencePose()); // reference pose
	fingerActorDesc.nxActorDesc.globalPose.M.setRowMajor(&zeroPose.R._m._11);
	fingerActorDesc.nxActorDesc.globalPose.t.set(&zeroPose.p.v1);
	
	Mat34 pose;
	pose.setId();
	pose.p.v2 = Real(0.1);
	pose.multiply(pose, arm.getReferencePose());
	arm.setReferencePose (pose);
	
	NxMat34 nxZeroPose(false);
	nxZeroPose.t.set(&zeroPose.p.v1);
	nxZeroPose.M.setRowMajor(&zeroPose.R._m._11);
	
	// create finger Actor
	Actor *pFingerActor = dynamic_cast<Actor*>(physReacPlanner.getScene().createObject(fingerActorDesc));
	if (pFingerActor == NULL) {
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_CRIT,
						    "FingerActor Object could not be created"
						    ));
		return;
	}

	Actor *effector = physReacPlanner.getJointActors()[jointIndex];
	// take the controller end-effector Actor
	if (effector == NULL) {
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_CRIT,
						    "End-effector has no body"
						    ));
		return;
	}

	const NxU32 solverAccuracy = (NxU32)255;
	const U32 numOfJointActors = physReacPlanner.getJointActors().size();

	fingerActorDesc.nxActorDesc.globalPose.multiply(nxZeroPose, fingerActorDesc.nxActorDesc.globalPose);
	pFingerActor->getNxActor()->raiseBodyFlag(NX_BF_DISABLE_GRAVITY);
	pFingerActor->getNxActor()->setSolverIterationCount(solverAccuracy);
   
		
	// make the finger a part of the arm body, but not a part rigidly attached to the last joint
	const U32 group = physReacPlanner.getArmBoundsGroup();
	pFingerActor->setBoundsGroup(group);
	for (NxArray<NxShapeDesc*, NxAllocatorDefault>::const_iterator i = fingerActorDesc.nxActorDesc.shapes.begin(); i != fingerActorDesc.nxActorDesc.shapes.end(); i++) {
		Bounds::Desc::Ptr pBoundsDesc =  physReacPlanner.getScene().createBoundsDesc(**i);
		if (pBoundsDesc != NULL) {
			pBoundsDesc->group = group;
			pBoundsDesc->pose.multiply(pBoundsDesc->pose, arm.getReferencePose());
			arm.Arm::addBoundsDesc(numOfJointActors - 1, pBoundsDesc);// HACK: call only Arm::addBoundsDesc() so the bounds will be invisible
		}
	}

	physReacPlanner.getPlanner().getHeuristic()->syncArmBoundsDesc(); // sync new arm bounds
	
	NxVec3 anchor(0.0f);
	fingerActorDesc.nxActorDesc.globalPose.multiply(anchor, anchor);
	NxVec3 axis(NxReal(0.0), NxReal(1.0), NxReal(0.0)); // along Y-axis
	fingerActorDesc.nxActorDesc.globalPose.M.multiply(axis, axis);

	NxJoint *pJoint = setupJoint(effector->getNxActor(), pFingerActor->getNxActor(), anchor, axis);
	if (pJoint == NULL) {
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_CRIT, "setupJoint(): Unable to create joint"));
	}

};





//function for normalizing values according to given bounds (before storing)
Real normalize(const Real& value, const Real& min, const Real& max) {
	Real val;
	if (min == -MATH_PI && max == MATH_PI && (value > max || value < min)) {
		val = fmod(value, MATH_PI);
	}
	else {
		val = value;
	}
	Real interval = max - min;
	Real relativeVal = val - min;
	Real res = relativeVal/interval;
	return -1.0 + (res*2.0);
}
	


//function that checks if arm hitted the polyflap while approaching it
bool checkPfPosition(const Actor* polyFlapActor, const Vec3& refPos1, const Vec3& refPos2) {
	
	Vec3 realPos1 = polyFlapActor->getBounds()->get().front()->getPose().p;
	Vec3 realPos2 = polyFlapActor->getBounds()->get().back()->getPose().p;
	
	return	abs(refPos1.v1 - realPos1.v1) < 0.00001 &&
		abs(refPos1.v2 - realPos1.v2) < 0.00001 &&
		abs(refPos2.v1 - realPos2.v1) < 0.00001 &&
		abs(refPos2.v2 - realPos2.v2) < 0.00001;

} 



void setMovementAngle(const int angle, msk::ctrl::WorkspaceCoord& pose,const Real& distance,const Vec3& normVec,const Vec3& orthVec) {
	pose.p.v1 += (sin(angle/180.0*REAL_PI)*(distance*normVec.v1)); 
	pose.p.v2 += (sin(angle/180.0*REAL_PI)*(distance*normVec.v2)); 
	pose.p.v1 += (cos(angle/180.0*REAL_PI)*(distance*orthVec.v1)); 
	pose.p.v2 += (cos(angle/180.0*REAL_PI)*(distance*orthVec.v2)); 
	pose.p.v3 += 0.0;	
}



Vec3 computeOrthogonalVec(const Vec3& normalVec) {
	Vec3 orthogonalVec(Real(normalVec.v2), Real(-1.0*normalVec.v1), Real(0.0));
	return orthogonalVec; 
}



Vec3 computeNormalVector(const Vec3& vector1, const Vec3& vector2) {
	Vec3 res(Real((vector2.v1 - vector1.v1)
			/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2))),
		Real((vector2.v2 - vector1.v2)
			/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2))),
		Real((vector2.v3 - vector1.v3)
			/sqrt(pow(vector2.v1 - vector1.v1,2) + pow(vector2.v2 - vector1.v2,2) + pow(vector2.v3 - vector1.v3,2))));
	return res;

}


void setPointCoordinates(Vec3& position, const Vec3& normalVec, const Vec3& orthogonalVec, const Real& spacing, const Real& horizontal, const Real& vertical) {
	position.v1 += (spacing*normalVec.v1); 
	position.v2 += (spacing*normalVec.v2); 
	position.v1 += (horizontal*orthogonalVec.v1); 
	position.v2 +=(horizontal*orthogonalVec.v2); 
	position.v3 += vertical; 
}

void setCoordinatesIntoTarget(const int startPosition, Vec3& positionT,const Vec3& polyflapNormalVec, const Vec3& polyflapOrthogonalVec,const Real& dist, const Real& side, const Real& center, const Real& top) {


			//set it's coordinates into target
			switch (startPosition) {
			case 1: 
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, Real(0.0));
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front down left (1)"));
				break;

			case 2:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), Real(0.0));
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front down middle (2)"));
				break;

			case 3:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, Real(0.0));
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front down right (3)"));
				break;

			case 4:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front center left (4)"));
				break;

			case 5:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), center);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front center middle (5)"));
				break;

			case 6:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, center);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front center right (6)"));
				break;

			case 7:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, top);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front up left (7)"));
				break;

			case 8:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, Real(0.0), top);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front up middle (8)"));
				break;

			case 9:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, -side, top);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Front up right (9)"));
				break;

			case 10:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, Real(0.0));
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back down left (10)"));
				break;

			case 11:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), Real(0.0));
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back down middle (11)"));
				break;

			case 12:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, Real(0.0));
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back down right (12)"));
				break;

			case 13:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, center);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back center left (13)"));
				break;

			case 14:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), center);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back center middle (14)"));
				break;

			case 15:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, center);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back center right (15)"));
				break;

			case 16:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, side, top);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back up left (16)"));
				break;

			case 17:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, Real(0.0), top);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back up middle (17)"));
				break;

			case 18:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, -dist, -side, top);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Back up right (18)"));
				break;

			case 19:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, Real(0.0));
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Side down left (19)"));
				break;

			case 20:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, Real(0.0));
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Side down right (20)"));
				break;

			case 21:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, center);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Side center left (21)"));
				break;

			case 22:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, center);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Side center right (22)"));
				break;

			case 23:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), side, top);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Side up left (23)"));
				break;

			case 24:
				setPointCoordinates(positionT, polyflapNormalVec, polyflapOrthogonalVec, Real(0.0), -side, top);
				//context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Side up right (24)"));
				break;

			};

}



void writeDownCollectedData(DataSet data) {
	time_t rawtime;
	struct tm * timeinfo;
  	char buffer [12];

 	time ( &rawtime );
  	timeinfo = localtime ( &rawtime );

  	strftime (buffer,12,"%y%m%d%H%M",timeinfo);
  	puts(buffer);

	string name;
	name.append(buffer);

	
	write_dataset(name  , data);
	//DataSet savedData;
	//read_dataset(name, savedData);
	//print_dataset<double> (savedData);
}













//-------------------------------------------------------cout<<positionT.v1<<"pos"<<endl;-------------------------





int main(int argc, char *argv[]) {
	// Determine configuration file name
	std::string cfg;
	if (argc == 1) {
		// default configuration file name
		cfg.assign(argv[0]);
		size_t pos = cfg.rfind(".exe"); // Windows only
		if (pos != std::string::npos) cfg.erase(pos);
		cfg.append(".xml");
	}
	else
		cfg.assign(argv[1]);

	// Create XML parser and load configuration file
	XMLParser::Desc parserDesc;
	XMLParser::Ptr parser = parserDesc.create();
	if (!parser->load(FileReadStream(cfg.c_str()))) {
		printf("unable to load configuration file: %s\n", cfg.c_str());
		printf("%s <configuration_file>\n", argv[0]);
		return 1;
	}

	// Find program XML root context
	XMLContext* xmlContext = parser->getContextRoot()->getContextFirst("golem");
	if (xmlContext == NULL) {
		printf("unknown configuration file: %s\n", cfg.c_str());
		return 1;
	}

	// Create program context
	msk::Context::Desc contextDesc;
	setupContext(contextDesc, xmlContext);
	msk::Context::Ptr context = contextDesc.create();
	if (context == NULL) {
		printf("unable to create program context");
		return 1;
	}

	printf("Use the arrow keys to move the camera.\n");
	printf("Use the mouse to rotate the camera.\n");
	printf("Press p to pause simulations.\n");
	printf("Press pgup/pgdn/space to switch between simulations.\n");
	printf("Press v to show Actors reference frames.\n");
	printf("Use z, x, c to change randering mode.\n");
	printf("Use F1-F12 to display program specific debug information:\n");
	printf("\tF1 to display/hide the current destination pose.\n");
	printf("\tF2 to display/hide the current trajectory.\n");
	printf("\tF3 to display/hide the goal/desired pose.\n");
	printf("\tF4 to display/hide the global waypoint graph nodes.\n");
	printf("\tF5 to display/hide the global waypoint path.\n");
	printf("\tF6 to display/hide the local waypoint graph nodes.\n");
	printf("\tF7 to display/hide the local waypoint path.\n");
	printf("\tF8 to display/hide the optimised waypoint path.\n");
	printf("Press esc to exit.\n");
	
	// Stream all messages to std::cerr
	Streamer streamer(context->getLogger(), std::cout);
	// Throw exception if the message level at least LEVEL_CRIT but only from the current thread
	context->getLogger()->setExFilter(MessageFilter::Ptr(new ThreadFilter<StdMsg>(StdMsg::LEVEL_CRIT)));
	// Do not display LEVEL_DEBUG messages (only with level at least LEVEL_INFO)
	//context->getLogger()->setInpFilter(MessageFilter::Ptr(new LevelFilter<StdMsg>(StdMsg::LEVEL_ERR)));

	// Random number generator seed
	context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Random number generator seed %d", context->getRandSeed()._U32[0]));

	//-----------------------------------------------------------------------------

	try {
		// Create Universe
		Universe::Desc universeDesc;
		setupUniverse(universeDesc, xmlContext->getContextFirst("universe"), *context);
		universeDesc.name = "Golem (Movement3)";
		universeDesc.argc = argc;
		universeDesc.argv = argv;
		Universe::Ptr pUniverse = universeDesc.create(*context);

		// Create scene
		Scene::Desc sceneDesc;
		setupScene(sceneDesc, xmlContext->getContextFirst("scene"), *context);
		sceneDesc.name = "Robotic arm controller demo";
		Scene *pScene = pUniverse->createScene(sceneDesc);

		// Determine arm type
		std::string armType;
		if (!XMLData(armType, xmlContext->getContextFirst("arm type"))) {
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_CRIT, "Unspecified arm type"));
			return 1;
		}
		













		// Setup PhysReacPlanner controller description
		obj_ptr<Object::Desc> pPhysReacPlannerDesc;
		if (!armType.compare("kat_serial_arm")) {
			typedef PhysReacPlanner::Desc<KatSerialArm, MyPRMPlanner, ReacPlanner> PhysReacPlannerDesc;
// 			typedef PhysReacPlanner::Desc<KatSerialArm, PRMPlanner, ReacPlanner> PhysReacPlannerDesc;
			PhysReacPlannerDesc *pDesc = new PhysReacPlannerDesc();
			pPhysReacPlannerDesc.reset(pDesc);
			setupPlanner(*pDesc, xmlContext, *context);
			XMLData(pDesc->armDesc.cfgPath, xmlContext->getContextFirst("arm kat_serial_arm path"));
			XMLData(pDesc->armDesc.serialDesc.commPort, xmlContext->getContextFirst("arm kat_serial_arm comm_port"));
		}
		else if (!armType.compare("kat_sim_arm")) {
			typedef PhysReacPlanner::Desc<KatSimArm, MyPRMPlanner, ReacPlanner> PhysReacPlannerDesc;
// 			typedef PhysReacPlanner::Desc<KatSimArm, PRMPlanner, ReacPlanner> PhysReacPlannerDesc;
			PhysReacPlannerDesc *pDesc = new PhysReacPlannerDesc();
			pPhysReacPlannerDesc.reset(pDesc);
			setupPlanner(*pDesc, xmlContext, *context);

		}
		else if (!armType.compare("gen_sim_arm")) {
			typedef PhysReacPlanner::Desc<GenSimArm, MyPRMPlanner, ReacPlanner> PhysReacPlannerDesc;
// 			typedef PhysReacPlanner::Desc<GenSimArm, PRMPlanner, ReacPlanner> PhysReacPlannerDesc;
			PhysReacPlannerDesc *pDesc = new PhysReacPlannerDesc();
			pPhysReacPlannerDesc.reset(pDesc);
			setupPlanner(*pDesc, xmlContext, *context);
		}
		else {
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_CRIT, "Unknown arm type"));
			return 1;
		}

		// Create PhysReacPlanner
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Initialising reactive planner..."));
		PhysReacPlanner *pPhysReacPlanner = dynamic_cast<PhysReacPlanner*>(pScene->createObject(*pPhysReacPlannerDesc));
		if (pPhysReacPlanner == NULL) {
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_CRIT, "Unable to create ReacPlanner"));
			return 1;
		}

		if (!armType.compare("kat_sim_arm")) {
			// Create finger to be attached to the 5th joint 
			const U32 jointIndex = 4;
			std::vector<Bounds::Desc::Ptr> bounds;
			MemoryWriteStream buffer;
			Mat34 pose;
			pose.setId();
			createFinger(bounds, pose/*arm.getReferencePose()*/, buffer);
			// Modify shape of the joint by adding a new Actor.
			addFinger(*pPhysReacPlanner, jointIndex, bounds, context);
		}

		













		// some useful pointers
		ReacPlanner &reacPlanner = pPhysReacPlanner->getReacPlanner();
		Planner &planner = pPhysReacPlanner->getPlanner();
		Arm &arm = pPhysReacPlanner->getArm();

/*		
// 		Mat34 p = pRobot->getFinger()->getFingerActor().getBounds()->get().front()->getPose();
		Real roll, pitch, yaw;
// 		p.R.toEuler (roll,pitch,yaw);
// 		context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Getting finger bounds pose..."));
// 		context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw));

		Mat34 p;
		//arm.setReferencePose (p);
		//arm.setReferencePose (arm.getGlobalPose());
		p = arm.getReferencePose ();
		p.R.toEuler (roll, pitch, yaw);
		
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Getting arm reference pose..."));
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw));

		p = arm.getGlobalPose ();
		p.R.toEuler (roll, pitch, yaw);
		
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Getting arm global pose..."));
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "%f, %f, %f, %f, %f, %f", p.p.v1, p.p.v2, p.p.v3, roll, pitch, yaw));
*/
		
		// Display arm information
		armInfo(arm);
		//sleep (1);








/////////////////////////////////////////////
///DATA//////////////////////////////////////
/////////////////////////////////////////////		


		//number of loop runs
		const int numExperiments = 1000;

		//a number that slightly greater then the maximal reachable space of the arm
		//    - used for workspace position normalization and later as a position upper bound
		//      for random polyflap position
		const Real maxRange = 0.7;

		//minimal duration of a movement (by normal speed)
		const SecTmReal minDuration = SecTmReal(5.0);
	
		//Polyflap Position and orientation
		const Vec3 startPolyflapPosition(Real(0.2), Real(0.2), Real(0.0));
		const Vec3 startPolyflapRotation(Real(-0.0*REAL_PI), Real(-0.0*REAL_PI), Real(-0.0*REAL_PI));//Y,X,Z
		//Polyflap dimensions		
		const Vec3 polyflapDimensions(Real(0.1), Real(0.1), Real(0.1)); //w,h,l
		

		//vertical distance from the ground
		const Real over = 0.01;
		//distance from the front/back of the polyflap
		const Real dist = 0.05;
		//distance from the side of the polyflap
		const Real side = polyflapDimensions.v1*0.6;
		//center of the poolyflop
		const Real center = polyflapDimensions.v2*0.5;
		//distance from the top of the polyflap
		const Real top = polyflapDimensions.v2* 1.2;
		//lenght of the movement		
		const Real distance = 0.2;

		
		//Dataset in which all sequences are stored; sequences and featureVectors are created
		//in every loop run
		DataSet data;

		//getting the maximal and minimal Velocities of arm joints	
		const U32 numOfJoints = (U32)arm.getJoints().size();
		float minVelocities [numOfJoints];
		float maxVelocities [numOfJoints];
		for (U32 i = 0; i < numOfJoints; i++) {
			const Joint &joint = *arm.getJoints()[i];
			minVelocities[i] = joint.getMin().vel;
		}
		for (U32 i = 0; i < numOfJoints; i++) {
			const Joint &joint = *arm.getJoints()[i];
			maxVelocities[i] = joint.getMax().vel;
		}


////////////////////////////////////////////
////////////////////////////////////////////
////////////////////////////////////////////







		// Big Bang!
		context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Launching Universe..."));
		pUniverse->launch();

		// Reactive arm controller is capable to make the arm to move on almost arbirtary trajectories
		// (within velocity and acceleration limits) using planner with collision detection
		// Trajectories are created by sequential sending trajectory waypoints to the arm
		// Waypoints can be specified in jointspace or in Cartesian workspace
		// Each waypoints has its own time stamp, and the arm controller takes care of time synchronization between
		// the arm and the program
		
		// Program has its own local time which is the same for all threads
		// and it is the time that has elapsed since the start of the program
		// Each arm controller has two characteristic time constants:
		// (Synchronous) Time Delta - a minimum elapsed time between two consecutive waypoints sent to the controller
		SecTmReal timeDelta = reacPlanner.getTimeDelta();
		// Asynchronous Time Delta - a minimum elapsed time between the current time and the first waypoint sent to the controller
		// (no matter what has been sent before)
		SecTmReal timeDeltaAsync = reacPlanner.getTimeDeltaAsync();

		
		// Define the Home pose in the Cartesian workspace
		Vec3 positionH(Real(0.0), Real(0.1), Real(0.1));
		Vec3 orientationH(Real(-0.5*MATH_PI), Real(0.0*MATH_PI), Real(0.0*MATH_PI));
		
		// and set target waypoint
		msk::ctrl::GenWorkspaceState home;
		fromCartesianPose(home.pos, positionH, orientationH);
		home.vel.setId(); // it doesn't move
		
		home.t = context->getTimer()->elapsed() + timeDeltaAsync + minDuration; // i.e. the movement will last at least 5 sec


		// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
		reacPlanner.send(home, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action (until the arm moves to the initial pose)
		reacPlanner.wait();



		// Define the initial pose in the Cartesian workspace
		Vec3 orientationT(Real(-0.5*MATH_PI), Real(0.0*MATH_PI), Real(0.0*MATH_PI));



		//start of the experiment loop
		for (int e=0; e<numExperiments; e++)
		{

			//polyflap actor
			Actor *polyFlapActor;
			context->getTimer()->sleep(1);
			{
				CriticalSectionWrapper csw(pScene->getUniverse().getCS());
				polyFlapActor = setupObjects(*pScene, startPolyflapPosition, startPolyflapRotation, polyflapDimensions, *context);
			}
			context->getTimer()->sleep(1);


			//reference polyflap position for prooving if arm didn't hit it whil approaching
			Vec3 referencePolyflapPosVec1 = polyFlapActor->getBounds()->get().front()->getPose().p;
			Vec3 referencePolyflapPosVec2 = polyFlapActor->getBounds()->get().back()->getPose().p;

			/////////////////////////////////////////////////
			//create sequence for this loop run and initial vector
			Sequence seq;
			FeatureVector infoVector;
			/////////////////////////////////////////////////

			msk::obj_ptr<msk::BoundsSet> curPol;
			Mat34 curPolPos1;
			Mat34 curPolPos2;	
			//find out bounds of polyflap and compute the position of the polyflap
			{
				CriticalSectionWrapper csw(pScene->getUniverse().getCS());
				curPol = polyFlapActor->getBounds();
			}
			if (curPol->get().front()->getPose().p.v3 > curPol->get().back()->getPose().p.v3) {
				curPolPos1 = curPol->get().front()->getPose();
				curPolPos2 = curPol->get().back()->getPose();
			}
			else {
				curPolPos1 = curPol->get().back()->getPose();
				curPolPos2 = curPol->get().front()->getPose();
			}

			Vec3 polyflapPosition(curPolPos1.p.v1, curPolPos1.p.v2, curPolPos2.p.v3);

			//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
			Vec3 polyflapNormalVec =
				computeNormalVector(
						    Vec3 (curPolPos1.p.v1, curPolPos1.p.v2, Real(0.0)),
						    Vec3 (curPolPos2.p.v1, curPolPos2.p.v2, Real(0.0))
						    );
			
			Vec3 polyflapOrthogonalVec = computeOrthogonalVec(polyflapNormalVec);	


			//initial target of the arm: the center of the polyflap
			Vec3 positionT(Real(polyflapPosition.v1), Real(polyflapPosition.v2), Real(polyflapPosition.v3 + over));


			//chose random point int the vicinity of the polyflap
			srand(context->getRandSeed()._U32[0]  + e);
			int startPosition = rand() % 17 + 1;
	
			setCoordinatesIntoTarget(startPosition, positionT, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center, top);
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Position %i", startPosition));


			// and set target waypoint
			msk::ctrl::GenWorkspaceState target;
			fromCartesianPose(target.pos, positionT, orientationT);
			target.vel.setId(); // it doesn't mov

			//target.t = context->getTimer()->elapsed() + timeDeltaAsync + SecTmReal(5.0); // i.e. the movement will last at least 5 sec
			target.t = context->getTimer()->elapsed() + timeDeltaAsync + minDuration; // i.e. the movement will last at least 5 sec

			for (int t=0; t<MAX_PLANNER_TRIALS; t++) {
				if (reacPlanner.send(target , ReacPlanner::ACTION_GLOBAL)) {
					break;
				}
				context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Unable to find path to polyflap, trying again."));
			}

			// wait for completion of the action (until the arm moves to the initial pose)
			reacPlanner.wait();
		
			// Trajectory profile can be defined by e.g. a simple 3rd degree polynomial
			Polynomial4::Desc polynomDesc;
			Trajectory::Ptr pTrajectory(/*Polynomial4::Desc().create()*/ polynomDesc.create());

			//initializing infoVector
			Polynomial4 polynom;
			polynom.create(polynomDesc);
			const Real* coefs  = polynom.getCoeffs();
			
			/////////////////////////////////////////////////
			//writing in the initial vector
			//Trajectory curve coefficients
			infoVector.push_back(normalize(coefs[0], -5, 5));
			infoVector.push_back(normalize(coefs[1], -5, 5));
			infoVector.push_back(normalize(coefs[2], -5, 5));
			infoVector.push_back(normalize(coefs[3], -5, 5));
			//initial position, normalized
			infoVector.push_back(normalize(positionT.v1, -maxRange, maxRange));
			infoVector.push_back(normalize(positionT.v2, -maxRange, maxRange));
			infoVector.push_back(normalize(positionT.v3, -maxRange, maxRange));
			//innitial orientation, normalized
			infoVector.push_back(normalize(orientationT.v1, -MATH_PI, MATH_PI));
			infoVector.push_back(normalize(orientationT.v2, -MATH_PI, MATH_PI));
			infoVector.push_back(normalize(orientationT.v3, -MATH_PI, MATH_PI));
			//end pose info missing (must be added later 
			/////////////////////////////////////////////////


			// It consists of 70 parts
			U32 n = 70;
			// Trajectory duration is a multiplicity of Time Delta [sec], but with a speed coefficient
			int speed = -1 + (rand() % 3);
			SecTmReal duration = timeDelta * n*(pow(2, (-speed)*2));
				
			/////////////////////////////////////////////////
			//writing in the initial vector
			infoVector.push_back(Real(speed));
			/////////////////////////////////////////////////

			// Trajectory end pose equals begin + shift along Y axis
			WorkspaceCoord begin = target.pos, end = target.pos;

			//normal vector to the center of the polyflap
			Vec3 polyflapCenterNormalVec =
				computeNormalVector(
						    Vec3 (positionT.v1, positionT.v2, positionT.v3),
						    Vec3 (polyflapPosition.v1, polyflapPosition.v2, polyflapDimensions.v2*0.5)
						    );
			//and it's orthogonal
			Vec3 polyflapCenterOrthogonalVec = computeOrthogonalVec(polyflapCenterNormalVec);


			//the lenght of the movement
			Real currDistance = distance;
			if (speed < 0) {
				currDistance *= 5.0;
			}


			//chose random horizontal and vertical angle
			int horizontalAngle = rand() % 61 + 60;
			
			//int verticalAngle = rand() % 7;

			setMovementAngle(horizontalAngle, end, currDistance, polyflapCenterNormalVec, polyflapCenterOrthogonalVec);
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "%d degree horizontaly", horizontalAngle));

			/////////////////////////////////////////////////
			//writing in the initial vector
			//add info about end position
			infoVector.push_back(normalize(end.p.v1, -maxRange, maxRange));
			infoVector.push_back(normalize(end.p.v2, -maxRange, maxRange));
			infoVector.push_back(normalize(end.p.v3, -maxRange, maxRange));
			//end orientation, normalized
			infoVector.push_back(normalize(orientationT.v1, -MATH_PI, MATH_PI));
			infoVector.push_back(normalize(orientationT.v2, -MATH_PI, MATH_PI));
			infoVector.push_back(normalize(orientationT.v3, -MATH_PI, MATH_PI));
			/////////////////////////////////////////////////

			/////////////////////////////////////////////////
			//writing of the initial vector into sequence
			seq.push_back(infoVector);
			/////////////////////////////////////////////////


			// Current time is the same for all threads and it is the time that has elapsed since the start of the program
			SecTmReal timeBegin = context->getTimer()->elapsed();
		
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Moving on the line..."));
			

			// ON/OFF collision detection
			planner.getHeuristic()->setCollisionDetection(false);



			
			//if the arm didn't touch the polyflap when approaching, we can proceed with the experiment
			//otherwise we skip it
			if (checkPfPosition(polyFlapActor, referencePolyflapPosVec1, referencePolyflapPosVec2)) {

			
				// Generate and send a simple straight line trajectory
				for (U32 i = 0; i <= n; i++) {

					// create a new target waypoint on a straight line at normalized time timeDelta * i
					waypointFromLineTrajectory(target, *pTrajectory, begin, end, duration, timeDelta * i);
					// set the target waypoint absolute time (in future)
					target.t = timeBegin + timeDeltaAsync + timeDelta * i;
		
					// send to the controller, force local movement (without planning in the entire arm workspace)
					// reacPlanner.wait() will wait approx 'timeDelta' seconds, until a next waypoint can be sent
					// reacPlanner.wait() can be used only if signal synchronization is enabled (see 'setupPlanner()' above)
					if (!reacPlanner.send(target, ReacPlanner::ACTION_LOCAL) || !reacPlanner.wait()) {
						// woops something went wrong
					}


					reacPlanner.wait();


					// arm state at a time t and finishes at some time later
					msk::ctrl::GenJointState state;
					arm.lookupInp(state, target.t); // last sent trajectory waypoint
			
					//to get polyflap pose information
					msk::obj_ptr<msk::BoundsSet> set;
					Mat34 mojepose2;
					Mat34 mojepose3;
					{
						CriticalSectionWrapper csw(pScene->getUniverse().getCS());
						set = polyFlapActor->getBounds();
					}
					//reading out the position of the polyflap
					mojepose2 = set->get().front()->getPose();
					mojepose3 = set->get().back()->getPose();
					
					/////////////////////////////////////////////////
					//creating current featureVector
					FeatureVector& features = *(new FeatureVector);
					/////////////////////////////////////////////////

					/////////////////////////////////////////////////
					//writing in the feature vector
					// joint position and joint velocity
					for (U32 i = 0; i < numOfJoints; i++) {
						features.push_back(normalize(state.pos[i], -MATH_PI, MATH_PI));
						features.push_back(normalize(state.vel.j[i], minVelocities[i], maxVelocities[i]));
					}
					/////////////////////////////////////////////////

					/////////////////////////////////////////////////
					//writing in the feature vector
					//the position of the polyflap (position of both sides separately)
					features.push_back(normalize(mojepose2.p.v1, -maxRange, maxRange));
					features.push_back(normalize(mojepose2.p.v2, -maxRange, maxRange));
					features.push_back(normalize(mojepose2.p.v3, -maxRange, maxRange));
					features.push_back(normalize(mojepose3.p.v1, -maxRange, maxRange));
					features.push_back(normalize(mojepose3.p.v2, -maxRange, maxRange));
					features.push_back(normalize(mojepose3.p.v3, -maxRange, maxRange));
					/////////////////////////////////////////////////

					/////////////////////////////////////////////////
					//writing the feature vector in the sequence
					seq.push_back(features);
					/////////////////////////////////////////////////

	
 
					//end of the movement
				}
				/////////////////////////////////////////////////
				//writing the sequence into the dataset
				data.push_back(seq);
				/////////////////////////////////////////////////



				// wait until the arm stops
				context->getTimer()->sleep(timeDeltaAsync - timeDelta);



				//end of the if(checkPosition...) block
			}


			// ON/OFF collision detection
			planner.getHeuristic()->setCollisionDetection(true);


				


			Vec3 positionPreH(target.pos.p.v1, target.pos.p.v2, target.pos.p.v3 += (polyflapDimensions.v2*1.1));
			// and set target waypoint
			msk::ctrl::GenWorkspaceState preHome;
			fromCartesianPose(preHome.pos, positionPreH, orientationH);
			home.vel.setId(); // it doesn't move

			home.t = context->getTimer()->elapsed() + timeDeltaAsync + minDuration; // i.e. the movement will last at least 5 sec

			// set the initial pose of the arm, force the global movement (with planning in the entire arm workspace)
			reacPlanner.send(preHome, ReacPlanner::ACTION_GLOBAL);
			// wait for completion of the action (until the arm moves to the initial pose)
			reacPlanner.wait();






			for (int t=0; t<MAX_PLANNER_TRIALS; t++) {
				if (reacPlanner.send(home, ReacPlanner::ACTION_GLOBAL)) {
					break;
				}
			
				context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Unable to find path home, trying again."));
			}





			context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Moving home..."));
			reacPlanner.wait();
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Done"));

			context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "trying to delete polyflap"));

			context->getTimer()->sleep(1);
			{
				CriticalSectionWrapper csw(pScene->getUniverse().getCS());
				pScene->releaseObject(*polyFlapActor);
				// wait a bit before new actor is created to avoid simulation crash
			}
			context->getTimer()->sleep(3);
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "deleting succeded"));
			context->getLogger()->post(StdMsg(StdMsg::LEVEL_INFO, "Iteration %d completed!", e));

				
		}// end of the for-loop (experiment loop)


	/////////////////////////////////////////////////
	//writing the dataset into binary file
	writeDownCollectedData(data);
	/////////////////////////////////////////////////


	}
	catch (const StdMsg &msg) {
		context->getLogger()->post(msg, false);
	}
	catch (const std::exception &ex) {
		context->getLogger()->post(StdMsg("C++ exception: %s", ex.what()));
	}

	return 0;
}
