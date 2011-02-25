package manipulation.strategies.parts.mobileManipulationNew.roationAwareNavigation;

import java.util.Observable;
import java.util.Observer;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.itemMemory.ItemMemory.ReachingStatus;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

public class GoToBestRotationalPoint extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the grasping approach part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public GoToBestRotationalPoint(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.GO_TO_BEST_ROTATIONAL_POINT);

	}

	public void alwaysGoToBestRotVP() {
		try {
			ViewPoint bestRotVP = (ViewPoint) getManipulator().getItemMemory()
					.getFirstGraspItem().getAttribute(
							PropertyName.BEST_ROTATIONAL_VIEWPOINT);

			try {
				getManipulator().getBaseConnector().goTo(
						bestRotVP.getPosition());
			} catch (NullPointerException e) {

				ViewPoint bestVP = (ViewPoint) getManipulator().getItemMemory()
						.getFirstGraspItem().getAttribute(
								PropertyName.BEST_VIEW_POINT);

				try {
					getManipulator().getBaseConnector().goTo(
							bestVP.getPosition());
				} catch (ExternalMemoryException e1) {
					logger.error(e1);
				}

				setNextPartName(PartName.ROTATIONAL_RECOGNIZE);

				synchronized (this) {
					notifyAll();
				}

			}
		} catch (ItemException e) {
			logger.error(e);
		} catch (InternalMemoryException e) {
			logger.error(e);
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		getManipulator().getItemMemory().addObserver(this);

		alwaysGoToBestRotVP();

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

	// TODO wenn keiner mehr da zu GOTOVP zurück!!!!
	// TODO auf rotieren achten

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
					setNextPartName(PartName.ROTATIONAL_RECOGNIZE);
					synchronized (this) {
						notifyAll();
					}
					break;
				case VP_NOT_REACHABLE:
					logger.error("not reachable");
					Item firstGraspItem = null;
					try {
						firstGraspItem = getManipulator().getItemMemory()
								.getFirstGraspItem();
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

							alwaysGoToBestRotVP();
						} else {
							// TODO zurückgehen und weitermachen
							// TODO was ist wenn der vorherige VP nun nicht mehr
							// erreichbar ist?

							ViewPoint bestVP = (ViewPoint) getManipulator()
									.getItemMemory().getFirstGraspItem()
									.getAttribute(PropertyName.BEST_VIEW_POINT);

							try {
								getManipulator().getBaseConnector().goTo(
										bestVP.getPosition());
							} catch (ExternalMemoryException e) {
								logger.error(e);
							}

							setNextPartName(PartName.ROTATIONAL_RECOGNIZE);

							synchronized (this) {
								notifyAll();
							}
						}
					} catch (ItemException e) {
						logger.error(e);
						setNextPartName(PartName.GO_TO_START_POSITION);
					} catch (InternalMemoryException e) {
						logger.error(e);
						setNextPartName(PartName.GO_TO_START_POSITION);

					}

					break;
				case OTHER:
					logger
							.error("An error occures in the navigation component");
					break;
				default:
					break;
				}

			}
		}

	}
}