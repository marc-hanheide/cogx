/**
 * 
 */
package springschool;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import mathlib.Functions;

import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Vector3;

import SpatialData.Place;
import VisionData.VisualObject;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;

/**
 * @author cogx
 * 
 */
public class SimpleRunner extends ManagedComponent {
	SimpleInterface simpleInterface;
	Set<Long> exploredIds = new HashSet<Long>();

	@Override
	protected void configure(Map<String, String> config) {
		// TODO Auto-generated method stub
		super.configure(config);
	}

	@Override
	protected void runComponent() {
		sleepComponent(5000);
		simpleInterface.executePTZCommand(0.0, -60 * Math.PI / 180);
		while (isRunning()) {
			try {
				Place nextPlace = determineNextPlace();
				if (nextPlace == null) {
					println("nothing more to explore :-( stopping here");
					break;
				}
				println("next place is " + nextPlace.id);
				simpleInterface.navigateToPlace(nextPlace);

				// do the turning
				for (int i = 0; i < 8; i++) {
					// double inRads = d * Math.PI * 180.0;
					println("look");
					VisualObject vo = simpleInterface
							.executeDetectCommand("example-cereals-schokomusli-sim");
					Pose3 objectPose = vo.pose;
					println("we found an object with conf "
							+ vo.identDistrib[0]);

					if (vo.identDistrib[0] > 0.00005) {
						println("reach out");

						doTheGrasp(objectPose);
						break;
					}

					println("turn");
					//simpleInterface.turnBy(45 * Math.PI / 180.0);

				}

			} catch (CASTException e) {
				logException(e);
			}
		}
	}

	private void doTheGrasp(Pose3 objectPose) {
		try {
			Pose3 closePoseInRobotFrame = objectPose;
			closePoseInRobotFrame.pos.x -= 0.6;
			Pose3 robotPoseInGlobalFrame = simpleInterface.getRobotPose();
			println("robot pose is " +Functions.toString(robotPoseInGlobalFrame));

			Pose3 navGoalPoseInGlobalFrame = Functions.transform(
					robotPoseInGlobalFrame, closePoseInRobotFrame);
			println("navGoalPoseInGlobalFrame is " +Functions.toString(navGoalPoseInGlobalFrame));
			simpleInterface.navigateToPos(navGoalPoseInGlobalFrame.pos.x,
					navGoalPoseInGlobalFrame.pos.y, simpleInterface
							.getRobotTheta());
			println("look again ");
			
			VisualObject vo = simpleInterface
					.executeDetectCommand("example-cereals-schokomusli-sim");
			if (vo.identDistrib[0] > 0.05) {
				println("found the object again, so we can grasp now");
				objectPose=new Pose3(new Vector3(), new Matrix33());
				objectPose.pos.x=vo.pose.pos.x;
				objectPose.pos.y=vo.pose.pos.y;
				objectPose.pos.z=vo.pose.pos.z;
				// try to grasp from above...
				objectPose.rot.m00=vo.pose.rot.m01;
				objectPose.rot.m10=vo.pose.rot.m11;
				objectPose.rot.m20=vo.pose.rot.m21;
				objectPose.rot.m01=-vo.pose.rot.m02;
				objectPose.rot.m11=-vo.pose.rot.m12;
				objectPose.rot.m21=-vo.pose.rot.m22;
				objectPose.rot.m02=vo.pose.rot.m00;
				objectPose.rot.m12=vo.pose.rot.m10;
				objectPose.rot.m22=vo.pose.rot.m20;

				
				println("reaching for " +Functions.toString(objectPose));
				simpleInterface.openGripper();
				simpleInterface.reachForPosition(objectPose.pos);
				simpleInterface.closeGripper();
				simpleInterface.moveArmHome();
				
			} else {
				println("object not found anymore");
			}

		} catch (CASTException e) {
			logException(e);
		}
	}

	Place determineNextPlace() throws CASTException {
		List<Place> places = new ArrayList<Place>();
		getMemoryEntries(Place.class, places, "spatial.sa");
		for (Place p : places) {
			log("existing place " + p.id);
			if (!exploredIds.contains(new Long(p.id))) {
				exploredIds.add(new Long(p.id));
				return p;
			}
		}
		return null;
	}

	@Override
	protected void start() {
		simpleInterface = new SimpleInterface(this);
	}

}
