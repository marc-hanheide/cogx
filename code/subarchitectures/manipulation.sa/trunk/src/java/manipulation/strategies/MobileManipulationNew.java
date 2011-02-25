package manipulation.strategies;

import java.util.Vector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Vector3D;
import manipulation.strategies.parts.StrategyPart.PartName;
import manipulation.strategies.parts.mobileManipulationNew.farNavigation.FarApproach;
import manipulation.strategies.parts.mobileManipulationNew.farNavigation.FarRecognize;
import manipulation.strategies.parts.mobileManipulationNew.farNavigation.GoToStartPosition;
import manipulation.strategies.parts.mobileManipulationNew.farNavigation.UpdateBestViewPointPosition;
import manipulation.strategies.parts.mobileManipulationNew.grasp.FarGrasping;
import manipulation.strategies.parts.mobileManipulationNew.grasp.FineGrasping;
import manipulation.strategies.parts.mobileManipulationNew.graspingAwareNavigation.CalculateBestGraspingPosition;
import manipulation.strategies.parts.mobileManipulationNew.graspingAwareNavigation.GoToBestGraspingPoint;
import manipulation.strategies.parts.mobileManipulationNew.graspingAwareNavigation.NearRecognize;
import manipulation.strategies.parts.mobileManipulationNew.roationAwareNavigation.GoToBestRotationalPoint;
import manipulation.strategies.parts.mobileManipulationNew.roationAwareNavigation.RotationalRecognize;
import manipulation.strategies.parts.mobileManipulationNew.roationAwareNavigation.UpdateBestViewPointRotation;

/**
 * defines the global strategy (state machine) to manipulate items with a mobile
 * robot
 * 
 * @author ttoenige
 * 
 */
public class MobileManipulationNew extends Strategy {

	private Vector<BasePositionData> objectSeen = new Vector<BasePositionData>();
	private Vector3D currentTarget;

	/**
	 * constructor of the strategy
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public MobileManipulationNew(Manipulator manipulator) {
		setManipulator(manipulator);
		setName(Name.MOBILE_MANIPULATION_NEW);
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
