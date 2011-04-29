package manipulation.strategies.parts;

import manipulation.core.share.Manipulator;
import manipulation.strategies.Strategy;

import org.apache.log4j.Logger;

/**
 * represent a part of the strategy (a state of the state machine)
 * 
 * @author Torben Toeniges
 * 
 */
public abstract class StrategyPart {
	private Manipulator manipulator;
	private PartName partName;
	private Strategy globalStrategy;
	private StrategyPart.PartName nextPartName;

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * names of the parts
	 * 
	 * @author Torben Toeniges
	 * 
	 */
	public enum PartName {
		/**
		 * 
		 */
		WAIT_PART,
		/**
		 * 
		 */
		FAR_ARM_MOVEMENT_COMMAND_PART,
		/**
		 * 
		 */
		PUT_DOWN_COMMAND_PART,
		/**
		 * 
		 */
		FINE_ARM_MOVEMENT_COMMAND_PART,
		/**
		 * 
		 */
		SIMULATE_FAR_ARM_MOVEMENT_PART,
		/**
		 * 
		 */
		LINEAR_BASE_MOVEMENT_APPROACH_COMMAND_PART,
		/**
		 * 
		 */
		STOP_COMMAND_PART,
		/**
		 * 
		 */
		MOVE_ARM_TO_HOME_POSITION_COMMAND_PART,
		/**
		 * 
		 */
		OPEN_GRIPPER_PART,
		/**
		 * 
		 */
		CLOSE_GRIPPER_PART,
		/**
		 * 
		 */
		MOVE_ARM_TO_POSE_PART,
		/**
		 * 
		 */
		GET_CURRENT_ARM_POSE_PART,
		/**
		 * 
		 */
		SIMULATE_MOVE_TO_POSE_PART
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
