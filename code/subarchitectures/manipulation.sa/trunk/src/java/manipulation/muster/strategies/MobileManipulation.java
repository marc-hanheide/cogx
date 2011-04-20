package manipulation.muster.strategies;

import java.util.Vector;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.types.BasePositionData;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.strategies.parts.StrategyPart.PartName;
import manipulation.muster.strategies.parts.mobileManipulation.farNavigation.FarApproach;
import manipulation.muster.strategies.parts.mobileManipulation.farNavigation.FarRecognize;
import manipulation.muster.strategies.parts.mobileManipulation.farNavigation.GoToStartPosition;
import manipulation.muster.strategies.parts.mobileManipulation.farNavigation.UpdateBestViewPointPosition;
import manipulation.muster.strategies.parts.mobileManipulation.grasp.FarGrasping;
import manipulation.muster.strategies.parts.mobileManipulation.grasp.FineGrasping;
import manipulation.muster.strategies.parts.mobileManipulation.graspingAwareNavigation.CalculateBestGraspingPosition;
import manipulation.muster.strategies.parts.mobileManipulation.graspingAwareNavigation.GoToBestGraspingPoint;
import manipulation.muster.strategies.parts.mobileManipulation.graspingAwareNavigation.NearRecognize;
import manipulation.muster.strategies.parts.mobileManipulation.roationAwareNavigation.GoToBestRotationalPoint;
import manipulation.muster.strategies.parts.mobileManipulation.roationAwareNavigation.RotationalRecognize;
import manipulation.muster.strategies.parts.mobileManipulation.roationAwareNavigation.UpdateBestViewPointRotation;

/**
 * defines the global strategy (state machine) to manipulate items with a mobile
 * robot
 * 
 * @author ttoenige
 * 
 */
public class MobileManipulation extends Strategy {

	private Vector<BasePositionData> objectSeen = new Vector<BasePositionData>();
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
		addToPartList(new UpdateBestViewPointPosition(getManipulator(), this));
		addToPartList(new UpdateBestViewPointRotation(getManipulator(), this));
		addToPartList(new GoToBestRotationalPoint(getManipulator(), this));
		addToPartList(new RotationalRecognize(getManipulator(), this));
		addToPartList(new CalculateBestGraspingPosition(getManipulator(), this));
		addToPartList(new GoToBestGraspingPoint(getManipulator(), this));
		addToPartList(new GoToStartPosition(getManipulator(), this));
		addToPartList(new NearRecognize(getManipulator(), this));
		addToPartList(new FarGrasping(getManipulator(), this));
		addToPartList(new FineGrasping(getManipulator(), this));
	}

	public void addToObejctSeenList(BasePositionData baseData) {
		objectSeen.add(baseData);
	}

	public boolean removeFromObjectSeenList(BasePositionData baseData) {
		return objectSeen.remove(baseData);
	}

	public Vector<BasePositionData> getObjectSeenList() {
		return objectSeen;
	}

	public void setCurrentTarget(Vector3D target) {
		this.currentTarget = target;
	}

	public Vector3D getCurrentTarget() {
		return currentTarget;
	}
}
