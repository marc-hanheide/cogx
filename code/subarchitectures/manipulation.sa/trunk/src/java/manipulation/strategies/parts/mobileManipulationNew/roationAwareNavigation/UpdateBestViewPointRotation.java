package manipulation.strategies.parts.mobileManipulationNew.roationAwareNavigation;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.MathException;
import manipulation.itemMemory.ItemMemory.UpdateAction;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

public class UpdateBestViewPointRotation extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the grasping approach part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public UpdateBestViewPointRotation(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.UPDATE_BEST_VIEWPOINT_ROTATION);

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		try {

			// TODO nur wenn VP geändertt wurde neu generieren!!!!!
			getManipulator().getItemMemory().updateRotationAwareViewPoints(
					getManipulator().getItemMemory().getFirstGraspItem(),
					getManipulator(), UpdateAction.GENERATE);

			setNextPartName(PartName.GO_TO_BEST_ROTATIONAL_POINT);

		} catch (ItemException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		} catch (MathException e) {
			logger.error(e);
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		logger.debug("we go on!");
		changeToNextPart();

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {
		logger.debug("cleaning up, starting next Part");

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}
}