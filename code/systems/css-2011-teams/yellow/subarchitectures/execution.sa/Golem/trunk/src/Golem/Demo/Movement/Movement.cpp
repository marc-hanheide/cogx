/** @file Movement.cpp
 * 
 * Demonstration program which moves the arm along a straight line
 * using reactive trajectory planner and collision detection
 * (also see DemoReacPlanner and DemoFinger).
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Phys/Application.h>
#include <Golem/Phys/Data.h>
#include <Golem/PhysCtrl/PhysReacPlanner.h>
#include <Golem/PhysCtrl/Creator.h>
#include <Golem/Demo/Common/Tools.h>

using namespace golem;

//------------------------------------------------------------------------------

void setupPlanner(PhysReacPlanner::Desc &desc, XMLContext* xmlcontext, golem::Context* context) {
	// some planner parameter tuning
	desc.pPlannerDesc->pHeuristicDesc->distJointcoordMax[4] = Real(1.0)*REAL_PI;// last joint
	// Enable signal synchronization (default value)
	//desc.pReacPlannerDesc->signalSync = true;
}

//------------------------------------------------------------------------------

/** MyApplication */
class MyApplication : public Application {
protected:
	/** Runs MyApplication */
	virtual void run(int argc, char *argv[]) {
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
		
		// Random number generator seed
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);

		// Get arm driver name
		std::string driver;
		XMLData("driver", driver, xmlcontext()->getContextFirst("arm"));
		// Load driver and setup planner
		PhysReacPlanner::Desc physReacPlannerDesc;
		physReacPlannerDesc.pArmDesc = Arm::Desc::load(*context(), driver);
		setupPlanner(physReacPlannerDesc, xmlcontext(), context());

		// Create PhysReacPlanner
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising reactive planner...");
		PhysReacPlanner *pPhysReacPlanner = dynamic_cast<PhysReacPlanner*>(scene()->createObject(physReacPlannerDesc));
		if (pPhysReacPlanner == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create ReacPlanner");
		
		// some useful pointers
		ReacPlanner &reacPlanner = pPhysReacPlanner->getReacPlanner();
		Planner &planner = pPhysReacPlanner->getPlanner();
		Arm &arm = pPhysReacPlanner->getArm();
		
		// Display arm information
		armInfo(arm);

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
		
		// Setup target #1
		// Define the initial pose in the Cartesian workspace
		Vec3 position(Real(0.2), Real(0.2), Real(0.1));
		Vec3 orientation(Real(-0.5)*REAL_PI, Real(0.0), Real(0.0));
		golem::GenWorkspaceState target;
		fromCartesianPose(target.pos, position, orientation);
		target.vel.setZero(); // it doesn't move
		target.t = context()->getTimer().elapsed() + timeDeltaAsync + SecTmReal(10.0); // i.e. the movement will last at least 10 sec
		
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Moving to target #1 using global planner ...");
		
		// set a new target pose of the arm, and the global movement (with planning in the entire arm workspace)
		reacPlanner.send(target, ReacPlanner::ACTION_GLOBAL);
		// wait for completion of the action
		reacPlanner.waitForEnd();

		// Setup target #2
		target.pos.p.v1 -= Real(0.4); // shift along X axis
		target.t = context()->getTimer().elapsed() + timeDeltaAsync + SecTmReal(10.0); // i.e. the movement will last at least 10 sec
		
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Moving to target #2 on the line trajectory using local planner ...");
		
		// set a new target pose of the arm, and the local movement (line trajectory)
		reacPlanner.send(target, ReacPlanner::ACTION_LOCAL);
		// wait for completion of the action
		reacPlanner.waitForEnd();
		
		// Setup target #3
		// Trajectory end pose equals begin
		WorkspaceCoord begin = target.pos, end = target.pos;
		end.p.v1 += Real(0.4); // shift along X axis

		// Trajectory profile can be defined by e.g. a simple 3rd degree polynomial
		Profile::Ptr pProfile(Polynomial4::Desc().create());
		// It consists of e.g. 70 parts
		U32 n = 70;
		// Trajectory duration is a multiplicity of Time Delta [sec]
		SecTmReal duration = timeDelta * n;

		// Current time is the same for all threads and it is the time that has elapsed since the start of the program
		SecTmReal timeBegin = context()->getTimer().elapsed();
		
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Moving to target #3 on the line trajectory using reactive controller ...");
		
		// Generate and send a simple straight line trajectory
		for (U32 i = 0; i <= n; i++) {
			// create a new target waypoint on a straight line at normalized time timeDelta * i
			waypointFromLineTrajectory(target, *pProfile, begin, end, duration, timeDelta * i);
			// set the target waypoint absolute time (in future)
			target.t = timeBegin + timeDeltaAsync + timeDelta * i;
			
			// send to the controller, force local movement (without planning in the entire arm workspace)
			// reacPlanner.wait() will wait approx 'timeDelta' seconds, until a next waypoint can be sent
			// reacPlanner.wait() can be used only if signal synchronization is enabled (see 'setupPlanner()' above)
			if (!reacPlanner.send(target, ReacPlanner::ACTION_MOVE) || !reacPlanner.waitForEnd()) {
				// woops something went wrong
			}
		}
		
		// wait until the arm stops
		PerfTimer::sleep(timeDeltaAsync - timeDelta);

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
