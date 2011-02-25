package manipulation.strategies.parts.mobileManipulationNew.roationAwareNavigation;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;
import manipulation.strategies.MobileManipulationNew;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;
import manipulation.strategies.parts.StrategyPart.PartName;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to recognize the object from a far distance
 * 
 * @author ttoenige
 * 
 */
public class RotationalRecognize extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the fare recognize part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public RotationalRecognize(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.ROTATIONAL_RECOGNIZE);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		boolean success = false;

		try {
			success = getManipulator().getCamConnector().recognizeTrackItem(
					getManipulator().getItemMemory().getFirstGraspItem());

		} catch (ExternalMemoryException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		}

		// TODO muss das hier?!
		try {
			Thread.sleep(10000);
		} catch (InterruptedException e1) {
			logger.error(e1);
		}

		if (success) {
			logger.debug("Object has been recognized!");

			try {
				double distance = MathOperation.getDistance(
						((MobileManipulationNew) getGlobalStrategy())
								.getCurrentTarget(),
						(Vector3D) getManipulator().getItemMemory()
								.getFirstGraspItem().getAttribute(
										PropertyName.WORLD_POSITION));

				
				logger.error("DISTANCE = " + distance);
				if (distance > 0.2) {
					logger.error("adapt best viewpoint");
					setNextPartName(PartName.UPDATE_BEST_VIEWPOINT_POSITION);
					changeToNextPart();
				} else {
					logger.error("does not adapt best viewpoint");
				}

				try {
					((MobileManipulationNew) getGlobalStrategy())
							.setCurrentTarget((Vector3D) getManipulator()
									.getItemMemory().getFirstGraspItem()
									.getAttribute(PropertyName.WORLD_POSITION));
				} catch (ItemException e1) {
					logger.error(e1);
				} catch (InternalMemoryException e1) {
					logger.error(e1);
				}

			} catch (ItemException e) {
				logger.error(e);
			} catch (InternalMemoryException e) {
				logger.error(e);
			}
			setNextPartName(PartName.CALCULATE_BEST_GRASPING_POINT);

		} else {
			Item firstGraspItem = null;

			try {
				firstGraspItem = getManipulator().getItemMemory()
						.getFirstGraspItem();
			} catch (InternalMemoryException e) {
				logger.error(e);
			}

			try {
				if (!((ViewPoints) firstGraspItem
						.getAttribute(PropertyName.ROTATIONAL_VIEWPOINT))
						.getPoints().isEmpty()) {

					getManipulator()
							.getItemMemory()
							.deleteRotationalVP(
									firstGraspItem,
									(ViewPoint) firstGraspItem
											.getAttribute(PropertyName.BEST_ROTATIONAL_VIEWPOINT));

					setNextPartName(PartName.GO_TO_BEST_ROTATIONAL_POINT);

				} else {
					getManipulator().getItemMemory().removeViewPoint(
							getManipulator().getItemMemory()
									.getFirstGraspItem(),
							(ViewPoint) getManipulator().getItemMemory()
									.getFirstGraspItem().getAttribute(
											PropertyName.BEST_VIEW_POINT),
							getManipulator());
					setNextPartName(PartName.FAR_APPROACH);
				}
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
