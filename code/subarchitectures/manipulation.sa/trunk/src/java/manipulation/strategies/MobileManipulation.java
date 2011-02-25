package manipulation.strategies;

import java.util.Vector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.types.ViewPoint;
import manipulation.strategies.parts.StrategyPart.PartName;
import manipulation.strategies.parts.mobileManipulation.CalculateGoToBestGraspingPosition;
import manipulation.strategies.parts.mobileManipulation.FarApproach;
import manipulation.strategies.parts.mobileManipulation.FarGrasping;
import manipulation.strategies.parts.mobileManipulation.FarRecognize;
import manipulation.strategies.parts.mobileManipulation.FineGrasping;
import manipulation.strategies.parts.mobileManipulation.UpdateBestViewPointPosition;
import manipulation.strategies.parts.mobileManipulation.UpdateBestViewPointRotation;
import manipulation.strategies.parts.mobileManipulation.GoToStartPosition;
import manipulation.strategies.parts.mobileManipulation.NearRecognize;

/**
 * defines the global strategy (state machine) to manipulate items with a mobile
 * robot
 * 
 * @author ttoenige
 * 
 */
public class MobileManipulation extends Strategy {

	private Vector<ViewPoint> objectSeen = new Vector<ViewPoint>();

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
		addToPartList(new FarGrasping(getManipulator(), this));
		addToPartList(new FarRecognize(getManipulator(), this));
		addToPartList(new NearRecognize(getManipulator(), this));
		addToPartList(new UpdateBestViewPointPosition(getManipulator(), this));
		addToPartList(new UpdateBestViewPointRotation(getManipulator(), this));
		addToPartList(new FineGrasping(getManipulator(), this));
		addToPartList(new GoToStartPosition(getManipulator(), this));
		addToPartList(new CalculateGoToBestGraspingPosition(getManipulator(),
				this));
	}

	public void addToObejctSeenList(ViewPoint viewPoint) {
		objectSeen.add(viewPoint);
	}

	public boolean removeFromObjectSeenList(ViewPoint viewPoint) {
		return objectSeen.remove(viewPoint);
	}
}
