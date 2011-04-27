package manipulation.muster.strategies;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.strategies.parts.StrategyPart.PartName;
import manipulation.muster.strategies.parts.mobileManipulation.farNavigation.FarApproach;
import manipulation.muster.strategies.parts.mobileManipulation.farNavigation.FarRecognize;
import manipulation.muster.strategies.parts.mobileManipulation.farNavigation.GoToStartPosition;
import manipulation.muster.strategies.parts.mobileManipulation.grasp.FarGrasping;
import manipulation.muster.strategies.parts.mobileManipulation.grasp.FineGrasping;
import manipulation.muster.strategies.parts.mobileManipulation.graspingAwareNavigation.GoToBestGraspingPoint;
import manipulation.muster.strategies.parts.mobileManipulation.graspingAwareNavigation.SimulateGrasp;

/**
 * defines the global strategy (state machine) to manipulate items with a mobile
 * robot
 * 
 * @author ttoenige
 * 
 */
public class MobileManipulation extends Strategy {

	private Vector3D currentTarget;

	/**
	 * constructor of the strategy
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public MobileManipulation(Manipulator manipulator) {
		setManipulator(manipulator);
		setName(Name.MOBILE_MANIPULATION);
		initParts();
		setFirstPart(getPart(PartName.FAR_APPROACH));

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void initParts() {
		addToPartList(new FarApproach(getManipulator(), this));
		addToPartList(new FarRecognize(getManipulator(), this));
		addToPartList(new SimulateGrasp(getManipulator(), this));
		addToPartList(new GoToBestGraspingPoint(getManipulator(), this));
		addToPartList(new GoToStartPosition(getManipulator(), this));
		addToPartList(new FarGrasping(getManipulator(), this));
		addToPartList(new FineGrasping(getManipulator(), this));
	}

	public void setCurrentTarget(Vector3D target) {
		this.currentTarget = target;
	}

	public Vector3D getCurrentTarget() {
		return currentTarget;
	}
}
