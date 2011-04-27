package manipulation.strategies;

import java.util.HashMap;
import java.util.Map;

import manipulation.core.share.Manipulator;
import manipulation.strategies.parts.StrategyPart;

/**
 * defines a strategy for the robot (state machine)
 * 
 * @author Torben Toeniges
 * 
 */
public abstract class Strategy {
	/**
	 * defines different names for the strategies
	 * 
	 * @author Torben Toeniges
	 * 
	 */
	public enum Name {
		/**
		 * 
		 */
		COMMAND_EXECUTION
	}

	private StrategyPart currentPart;
	private Map<StrategyPart.PartName, StrategyPart> strategyPartMap = new HashMap<StrategyPart.PartName, StrategyPart>();
	private Manipulator manipulator;
	private Name name;


	/**
	 * sets the next part of the strategy
	 * 
	 * @param nextPart
	 *            next part to execute
	 */
	public void setNextPart(StrategyPart nextPart) {
		this.currentPart = nextPart;
		nextPart.execute();
	}

	/**
	 * gets the part to the corresponding name
	 * 
	 * @param partName
	 *            name of the part to return
	 * @return strategy part
	 */
	public StrategyPart getPart(StrategyPart.PartName partName) {
		if (strategyPartMap.containsKey(partName)) {
			return strategyPartMap.get(partName);
		} else {
			// TODO throw machen und nicht so
			return null;
		}
	}

	/**
	 * add a strategy part to the strategy
	 * 
	 * @param part
	 *            part to add to the strategy
	 */
	public void addToPartList(StrategyPart part) {
		strategyPartMap.put(part.getPartName(), part);
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
	 *            manipulator to set
	 */
	public void setManipulator(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	/**
	 * initialize the strategy parts of the strategies
	 */
	public abstract void initParts();

	/**
	 * set the first part of the strategy
	 * 
	 * @param part
	 *            first part of the strategy
	 */
	public void setFirstPart(StrategyPart part) {
		currentPart = part;
	}

	/**
	 * start the execution of the global strategy -> execute first part
	 */
	public void startExecution() {
		currentPart.execute();
	}

	/**
	 * @return the name
	 */
	public Name getName() {
		return name;
	}

	/**
	 * @param name
	 *            the name to set
	 */
	public void setName(Name name) {
		this.name = name;
	}
	

}
