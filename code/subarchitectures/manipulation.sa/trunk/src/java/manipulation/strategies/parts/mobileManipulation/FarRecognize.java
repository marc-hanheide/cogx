package manipulation.strategies.parts.mobileManipulation;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.strategies.MobileManipulation;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to recognize the object from a far distance
 * 
 * @author ttoenige
 * 
 */
public class FarRecognize extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the fare recognize part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public FarRecognize(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FAR_RECOGNIZE);
	}

	ViewPoint bestVP = null;

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute - far recognize - ");

		boolean success = false;

		try {
			bestVP = (ViewPoint) getManipulator().getItemMemory()
					.getFirstGraspItem().getAttribute(
							PropertyName.BEST_VIEW_POINT);

			success = getManipulator().getCamConnector().recognizeTrackItem(
					getManipulator().getItemMemory().getFirstGraspItem());

		} catch (ExternalMemoryException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		} catch (ItemException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// TODO muss das hier?!
		try {
			Thread.sleep(10000);
		} catch (InterruptedException e1) {
			logger.error(e1);
		}

		if (success) {
			logger.debug("Recognizes and tracks object!");

			try {
				ViewPoint currentBestVP = (ViewPoint) getManipulator()
						.getItemMemory().getFirstGraspItem().getAttribute(
								PropertyName.BEST_VIEW_POINT);

				((MobileManipulation) getGlobalStrategy())
						.addToObejctSeenList(currentBestVP);
			} catch (ItemException e) {
				logger.error(e);
			} catch (InternalMemoryException e) {
				logger.error(e);
			}

			setNextPartName(PartName.UPDATE_BEST_VIEWPOINT_POSITION);

		} else {
			try {
				getManipulator().getItemMemory().removeViewPoint(
						getManipulator().getItemMemory().getFirstGraspItem(),
						(ViewPoint) getManipulator().getItemMemory()
								.getFirstGraspItem().getAttribute(
										PropertyName.BEST_VIEW_POINT), getManipulator());
				setNextPartName(PartName.FAR_APPROACH);
			} catch (ItemException e) {
				logger.error(e);
				setNextPartName(PartName.GO_TO_START_POSITION);
			} catch (InternalMemoryException e) {
				logger.error(e);
				setNextPartName(PartName.GO_TO_START_POSITION);
			}

		}

		changeToNextPart();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {

		try {
			getManipulator().getCamConnector().resetTracker();
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}
}
