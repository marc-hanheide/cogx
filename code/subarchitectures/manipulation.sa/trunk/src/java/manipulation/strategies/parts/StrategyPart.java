package manipulation.strategies.parts;

import manipulation.core.share.Manipulator;
import manipulation.strategies.Strategy;

/**
 * represent a part of the strategy (a state of the state machine)
 * 
 * @author ttoenige
 * 
 */
public abstract class StrategyPart {
	private Manipulator manipulator;
	private PartName partName;
	private Strategy globalStrategy;
	private StrategyPart.PartName nextPartName;

	/**
	 * names of the parts
	 * 
	 * @author ttoenige
	 * 
	 */
	public enum PartName {
		/**
		 * Name for the far approach part.
		 */
		FAR_APPROACH,
		/**
		 * Name for the far grasping part.
		 */
		FAR_GRASPING,
		/**
		 * Name for the far recognize part.
		 */
		FAR_RECOGNIZE,
		/**
		 * Name for the fine grasping part.
		 */
		FINE_GRASPING,
		/**
		 * Name for the far update best viewpoint position part.
		 */
		UPDATE_BEST_VIEWPOINT_POSITION,
		/**
		 * Name for the far update best viewpoint rotation part.
		 */
		UPDATE_BEST_VIEWPOINT_ROTATION,
		/**
		 * Name for the go to start position part.
		 */
		GO_TO_START_POSITION,
		/**
		 * Name for the go to virtual pose in real world part.
		 */
		GO_TO_VIRTUAL_POSE_IN_REAL_WORLD,
		/**
		 * Name for the near recognize part.
		 */
		NEAR_RECOGNIZE,
		/**
		 * Name for the calculate and goto best grasping point part.
		 */
		CALCULATE_GOTO_BEST_GRASPING_POINT,
		/**
		 * Name for the calibration part.
		 */
		CALIBRATION,
		/**
		 * 
		 */
		GO_TO_BEST_ROTATIONAL_POINT,
		/**
		 * 
		 */
		ROTATIONAL_RECOGNIZE, 
		/**
		 * 
		 */
		CALCULATE_BEST_GRASPING_POINT,
		/**
		 * 
		 */
		GO_TO_BEST_GRASPING_POINT
	}

	/**
	 * gets the current manipulator
	 * 
	 * @return current manipulator
	 */
	public Manipulator getManipulator() {
		return manipulator;
	}

	/**
	 * sets the current manipulator
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public void setManipulator(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	/**
	 * gets the part name
	 * 
	 * @return part name of the part
	 */
	public PartName getPartName() {
		return partName;
	}

	/**
	 * sets the part name
	 * 
	 * @param partName
	 *            part name to set
	 */
	public void setPartName(PartName partName) {
		this.partName = partName;
	}

	/**
	 * gets the global strategy
	 * 
	 * @return current used global strategy
	 */
	public Strategy getGlobalStrategy() {
		return globalStrategy;
	}

	/**
	 * sets the global strategy to use
	 * 
	 * @param globalStrategy
	 *            strategy to use
	 */
	public void setGlobalStrategy(Strategy globalStrategy) {
		this.globalStrategy = globalStrategy;
	}

	/**
	 * gets the next part name
	 * 
	 * @return next part name
	 */
	public StrategyPart.PartName getNextPartName() {
		return nextPartName;
	}

	/**
	 * defines the next part name
	 * 
	 * @param nextPartName
	 *            next part name
	 */
	public void setNextPartName(StrategyPart.PartName nextPartName) {
		this.nextPartName = nextPartName;
	}

	/**
	 * executes this strategy part
	 */
	public abstract void execute();

	/**
	 * changes to the next part of the global strategy
	 */
	public abstract void changeToNextPart();
}
