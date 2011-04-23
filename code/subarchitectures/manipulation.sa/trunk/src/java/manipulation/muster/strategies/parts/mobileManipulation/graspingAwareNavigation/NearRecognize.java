package manipulation.muster.strategies.parts.mobileManipulation.graspingAwareNavigation;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.exceptions.ExternalMemoryException;
import manipulation.muster.core.share.exceptions.InternalMemoryException;
import manipulation.muster.core.share.exceptions.ItemException;
import manipulation.muster.core.share.exceptions.ViewPointException;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.core.share.types.ViewPoint;
import manipulation.muster.core.share.types.ViewPoints;
import manipulation.muster.itemMemory.Item;
import manipulation.muster.itemMemory.Item.PropertyName;
import manipulation.muster.math.MathOperation;
import manipulation.muster.strategies.MobileManipulation;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;

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

		try {
			Thread.sleep(10000);
		} catch (InterruptedException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		logger.error("success: " + success);

		if (success) {
			logger.debug("Recognizes and tracks object!");

			try {
				double distance = MathOperation.getDistance(
						((MobileManipulation) getGlobalStrategy())
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
					((MobileManipulation) getGlobalStrategy())
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

			setNextPartName(PartName.FAR_GRASPING);
		} else {

//			try {
//				if (getManipulator().getVirtualSceneConnector()
//						.removeGraspingBasePoint(
//								getManipulator().getVirtualSceneConnector()
//										.getBestGraspingBasePoint())) {
//					logger.error("another try");
//					setNextPartName(PartName.GO_TO_BEST_GRASPING_POINT);
//				} else {
//					logger.error("No more Grasping point -> go on");
//
//					Item firstGraspItem = null;
//					try {
//						firstGraspItem = getManipulator().getItemMemory()
//								.getFirstGraspItem();
//					} catch (InternalMemoryException e) {
//						// TODO Auto-generated catch block
//						e.printStackTrace();
//					}
//
//					try {
//						if (!((ViewPoints) firstGraspItem
//								.getAttribute(PropertyName.ROTATIONAL_VIEWPOINT))
//								.getPoints().isEmpty()) {
//
//							getManipulator()
//									.getItemMemory()
//									.deleteRotationalVP(
//											firstGraspItem,
//											(ViewPoint) firstGraspItem
//													.getAttribute(PropertyName.BEST_ROTATIONAL_VIEWPOINT));
//
//							setNextPartName(PartName.GO_TO_BEST_ROTATIONAL_POINT);
//							synchronized (this) {
//								notifyAll();
//							}
//						} else {
//							getManipulator()
//									.getItemMemory()
//									.removeViewPoint(
//											firstGraspItem,
//											(ViewPoint) firstGraspItem
//													.getAttribute(PropertyName.BEST_VIEW_POINT),
//											getManipulator());
//
//							setNextPartName(PartName.FAR_APPROACH);
//
//							synchronized (this) {
//								notifyAll();
//							}
//
//						}
//					} catch (ItemException e) {
//						logger.error(e);
//					} catch (InternalMemoryException e) {
//						logger.error(e);
//					}
//				}
//			} catch (ViewPointException e) {
//				logger.error(e);
//			}

		}

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
