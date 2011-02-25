package manipulation.strategies.parts.mobileManipulation;

import java.util.Observable;
import java.util.Observer;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

public class UpdateBestViewPointRotation extends StrategyPart implements
		Observer {

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
		logger.debug("execute - update best viewpoint rotation - ");
		getManipulator().getItemMemory().addObserver(this);

		try {

			getManipulator().getItemMemory().updateViewPointErrorByAngle(
					getManipulator().getItemMemory().getFirstGraspItem(),
					getManipulator());

			double distance = MathOperation.getDistance(getManipulator()
					.getBaseConnector().getCurrentPosition().getPoint(),
					((ViewPoint) getManipulator().getItemMemory()
							.getFirstGraspItem().getAttribute(
									PropertyName.BEST_VIEW_POINT))
							.getPosition().getPoint());
			if (distance > 0.2) {
				logger.error("other ViewPoint");

				setNextPartName(PartName.FAR_APPROACH);
			} else {
				logger.error("good ViewPoint");
				setNextPartName(PartName.CALCULATE_GOTO_BEST_GRASPING_POINT);
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

		getManipulator().getItemMemory().deleteObserver(this);

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void update(Observable observable, Object arg) {
		if (observable instanceof ItemMemory) {

		}
	}
}