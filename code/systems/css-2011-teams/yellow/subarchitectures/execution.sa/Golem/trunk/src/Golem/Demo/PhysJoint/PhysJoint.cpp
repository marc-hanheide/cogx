/** @file PhysJoint.cpp
 * 
 * Demonstration program which moves the arm to the zero pose.
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the Katana arm simulator
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Phys/Application.h>
#include <Golem/PhysCtrl/PhysArm.h>
#include <Golem/Demo/Common/Tools.h>
#include <iostream>

using namespace golem;

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
		printf("Use F1-F12 to display program specific debug information.\n");
		printf("\tF1 to display/hide the current destination pose.\n");
		printf("\tF2 to display/hide the current trajectory.\n");
		printf("Press esc to exit.\n");

		// Get arm driver name
		std::string driver;
		XMLData("driver", driver, xmlcontext()->getContextFirst("arm"));
		// Load driver
		PhysArm::Desc physArmDesc;
		physArmDesc.pArmDesc = Arm::Desc::load(*context(), driver);

		// Create PhysArm controller
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Initialising arm controller...");
		PhysArm *pPhysArm = dynamic_cast<PhysArm*>(scene()->createObject(physArmDesc));
		if (pPhysArm == NULL)
			throw Message(Message::LEVEL_CRIT, "Unable to create arm controller");
		Arm &arm = pPhysArm->getArm();
		
		// Display arm information
		armInfo(arm);

		// Get the current position of joints
		GenConfigspaceState begin;
		if (!arm.recv(begin))
			throw Message(Message::LEVEL_CRIT, "armZeroMove(): receive error");
		
		// Generate trajectory end-point
		GenConfigspaceCoord end;
		// Zero joint position, velocity and acceleration
		end.set(GenCoord(REAL_ZERO, REAL_ZERO, REAL_ZERO));

		// Choose movement velocity and acceleration to 50%
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Moving to the ZERO pose");
		armMove(arm, end, Real(0.5), Real(0.5)); // zero pose
		armWatch(arm);

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Press the spacebar to continue");
		while (universe()->waitKey() != ' ' && !universe()->interrupted());

		// Choose movement velocity and acceleration to 50%
		context()->getMessageStream()->write(Message::LEVEL_INFO, "Moving to the INITIAL pose");
		armMove(arm, begin, Real(0.5), Real(0.5)); // and back
		armWatch(arm);

		context()->getMessageStream()->write(Message::LEVEL_INFO, "Good bye!");
	}
};

int main(int argc, char *argv[]) {
	return MyApplication().main(argc, argv);
}
