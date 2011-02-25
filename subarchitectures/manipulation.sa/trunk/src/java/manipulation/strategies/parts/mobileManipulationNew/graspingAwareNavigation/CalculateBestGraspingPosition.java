package manipulation.strategies.parts.mobileManipulationNew.graspingAwareNavigation;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

public class CalculateBestGraspingPosition extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the virtual fine approach part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public CalculateBestGraspingPosition(Manipulator manipulator,
			Strategy globalStrategy) {
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
		
		try {
			getManipulator().getSimulationConnector()
					.updateBestGraspingBasePoints(
							getManipulator().getBaseConnector()
									.getCurrentPosition(),
							getManipulator().getItemMemory()
									.getFirstGraspItem());

			setNextPartName(PartName.GO_TO_BEST_GRASPING_POINT);
			changeToNextPart();
		} catch (ItemException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		} catch (InternalMemoryException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		} catch (ExternalMemoryException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		}

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