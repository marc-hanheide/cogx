package manipulation.strategies.parts.mobileManipulationNew.farNavigation;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;
import manipulation.strategies.MobileManipulationNew;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

public class UpdateBestViewPointPosition extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the grasping approach part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public UpdateBestViewPointPosition(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.UPDATE_BEST_VIEWPOINT_POSITION);

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		try {
			getManipulator().getItemMemory().updateViewPointErrorByDistance(
					getManipulator().getItemMemory().getFirstGraspItem(),
					getManipulator());

			double distance = MathOperation.getDistance(getManipulator()
					.getBaseConnector().getCurrentPosition().getPoint(),
					((ViewPoint) getManipulator().getItemMemory()
							.getFirstGraspItem().getAttribute(
									PropertyName.BEST_VIEW_POINT))
							.getPosition().getPoint());

			// TODO constant
			if (distance > 0.1) {
				logger.error("Other and better viewpoint avialable!");
				logger.error("Dist: " + distance);
				setNextPartName(PartName.FAR_APPROACH);

			} else {
				logger.error("Reached good viewpoint");
				logger.error("Dist: " + distance);
				setNextPartName(PartName.UPDATE_BEST_VIEWPOINT_ROTATION);
			}
		} catch (ExternalMemoryException e) {
			logger.error(e);
		} catch (ItemException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
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