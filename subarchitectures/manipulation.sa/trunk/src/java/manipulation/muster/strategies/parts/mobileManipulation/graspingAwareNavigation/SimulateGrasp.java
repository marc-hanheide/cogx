package manipulation.muster.strategies.parts.mobileManipulation.graspingAwareNavigation;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

public class SimulateGrasp extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the virtual fine approach part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public SimulateGrasp(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.CALCULATE_BEST_GRASPING_POINT);

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {

		logger.error("execute: " + this.getClass());

		setNextPartName(PartName.GO_TO_BEST_GRASPING_POINT);
		changeToNextPart();

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {
		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}

}