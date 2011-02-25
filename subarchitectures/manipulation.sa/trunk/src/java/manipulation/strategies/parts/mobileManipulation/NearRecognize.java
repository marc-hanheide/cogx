package manipulation.strategies.parts.mobileManipulation;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to recognize an object if the robot is directly in front
 * of the object
 * 
 * @author ttoenige
 * 
 */
public class NearRecognize extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the near recognize part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public NearRecognize(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.NEAR_RECOGNIZE);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute - near recognize - ");

		boolean success = false;

		try {
			success = getManipulator().getCamConnector().recognizeTrackItem(
					getManipulator().getItemMemory().getFirstGraspItem());

		} catch (ExternalMemoryException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		}

		try {
			Thread.sleep(10000);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		logger.error("success: " + success);

		if (success) {
			logger.debug("Recognizes and tracks object!");

			setNextPartName(PartName.FAR_GRASPING);
		} else {
			try {
				getManipulator().getItemMemory().removeViewPoint(
						getManipulator().getItemMemory().getFirstGraspItem(),
						(ViewPoint) getManipulator().getItemMemory()
								.getFirstGraspItem().getAttribute(
										PropertyName.BEST_VIEW_POINT),getManipulator());
				setNextPartName(PartName.FAR_APPROACH);
				stopTracking();
			} catch (ItemException e) {
				logger.error(e);
				setNextPartName(PartName.GO_TO_START_POSITION);
				stopTracking();
			} catch (InternalMemoryException e) {
				logger.error(e);
				setNextPartName(PartName.GO_TO_START_POSITION);
				stopTracking();
			}

		}

		changeToNextPart();

	}

	private void stopTracking() {
		try {
			getManipulator().getCamConnector().resetTracker();
		} catch (ExternalMemoryException e) {
			logger.error(e);
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
