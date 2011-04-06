package manipulation.strategies;

import java.util.Vector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Vector3D;
import manipulation.strategies.parts.StrategyPart.PartName;
import manipulation.strategies.parts.mobileManipulation.farNavigation.FarApproach;
import manipulation.strategies.parts.mobileManipulation.farNavigation.FarRecognize;
import manipulation.strategies.parts.mobileManipulation.farNavigation.GoToStartPosition;
import manipulation.strategies.parts.mobileManipulation.farNavigation.UpdateBestViewPointPosition;
import manipulation.strategies.parts.mobileManipulation.grasp.FarGrasping;
import manipulation.strategies.parts.mobileManipulation.grasp.FineGrasping;
import manipulation.strategies.parts.mobileManipulation.graspingAwareNavigation.CalculateBestGraspingPosition;
import manipulation.strategies.parts.mobileManipulation.graspingAwareNavigation.GoToBestGraspingPoint;
import manipulation.strategies.parts.mobileManipulation.graspingAwareNavigation.NearRecognize;
import manipulation.strategies.parts.mobileManipulation.roationAwareNavigation.GoToBestRotationalPoint;
import manipulation.strategies.parts.mobileManipulation.roationAwareNavigation.RotationalRecognize;
import manipulation.strategies.parts.mobileManipulation.roationAwareNavigation.UpdateBestViewPointRotation;

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
