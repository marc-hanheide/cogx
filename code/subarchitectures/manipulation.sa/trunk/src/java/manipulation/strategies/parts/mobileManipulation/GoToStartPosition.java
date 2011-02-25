package manipulation.strategies.parts.mobileManipulation;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.types.BasePositionData;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to go to the home position of the robot (only base
 * movement)
 * 
 * @author ttoenige
 * 
 */
public class GoToStartPosition extends StrategyPart {
	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the go home part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public GoToStartPosition(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.GO_TO_START_POSITION);
	}

	private void goHome() {
		try {
			getManipulator().getBaseConnector().goTo(
					new BasePositionData(0, 0, 0));
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute - goto start position - ");
		goHome();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {
		// Nothing to do here
	}

}
