package manipulation.strategies.parts.mobileManipulationNew.graspingAwareNavigation;

import java.util.Observable;
import java.util.Observer;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ViewPointException;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.itemMemory.ItemMemory.ReachingStatus;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

public class GoToBestGraspingPoint extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the grasping approach part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public GoToBestGraspingPoint(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.GO_TO_BEST_GRASPING_POINT);

	}

	public void approach() {
		try {

			ViewPoint bestPoint = getManipulator().getSimulationConnector()
					.getBestGraspingBasePoint();

			logger.error("Best grasping point in front: " + bestPoint);
			logger.error("Go to best grasping point in front");

			try {
				getManipulator().getBaseConnector().goTo(
						bestPoint.getPosition());
			} catch (NullPointerException e) {
				logger.error("No more Graspingpoints - go On");

				Item firstGraspItem = null;
				try {
					firstGraspItem = getManipulator().getItemMemory()
							.getFirstGraspItem();
				} catch (InternalMemoryException e1) {
					logger.error(e1);
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
						logger.error("will nach GO_TO_BEST_ROTATIONAL");
						synchronized (this) {
							notifyAll();
						}
						return;
					} else {
						getManipulator()
								.getItemMemory()
								.removeViewPoint(
										firstGraspItem,
										(ViewPoint) firstGraspItem
												.getAttribute(PropertyName.BEST_VIEW_POINT),
										getManipulator());

						setNextPartName(PartName.FAR_APPROACH);

						logger.error("will nach FAR_APPROACH");
						synchronized (this) {
							notifyAll();
						}
						return;

					}
				} catch (ItemException e1) {
					logger.error(e1);
				} catch (InternalMemoryException e1) {
					logger.error(e1);
				}

			}

		} catch (ViewPointException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			synchronized (this) {
				notifyAll();
			}
		} catch (ExternalMemoryException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			synchronized (this) {
				notifyAll();
			}
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		getManipulator().getItemMemory().addObserver(this);

		approach();

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);

			}
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
			if (arg instanceof ReachingStatus) {
				logger.debug("Reaching status changed in memory to " + arg);
				ReachingStatus currentStatus = ((ReachingStatus) arg);

				switch (currentStatus) {
				case VP_REACHED:
					logger.error("reached");
					setNextPartName(PartName.NEAR_RECOGNIZE);
					synchronized (this) {
						notifyAll();
					}
					break;
				case VP_NOT_REACHABLE:
					logger.error("not reachable");

					try {
						if (getManipulator().getSimulationConnector()
								.removeGraspingBasePoint(
										getManipulator()
												.getSimulationConnector()
												.getBestGraspingBasePoint())) {
							logger.error("another try");
							// TODO best point neu berechnen?!?!
							approach();
						} else {
							logger.error("No more Grasping point -> go on");

							Item firstGraspItem = null;
							try {
								firstGraspItem = getManipulator()
										.getItemMemory().getFirstGraspItem();
							} catch (InternalMemoryException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
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
									synchronized (this) {
										notifyAll();
									}
								} else {
									getManipulator()
											.getItemMemory()
											.removeViewPoint(
													firstGraspItem,
													(ViewPoint) firstGraspItem
															.getAttribute(PropertyName.BEST_VIEW_POINT),
													getManipulator());

									setNextPartName(PartName.FAR_APPROACH);

									synchronized (this) {
										notifyAll();
									}

								}
							} catch (ItemException e) {
								logger.error(e);
							} catch (InternalMemoryException e) {
								logger.error(e);
							}

						}
					} catch (ViewPointException e) {
						logger.error(e);
					}
					break;
				default:
					break;
				}

			}
		}

	}
}