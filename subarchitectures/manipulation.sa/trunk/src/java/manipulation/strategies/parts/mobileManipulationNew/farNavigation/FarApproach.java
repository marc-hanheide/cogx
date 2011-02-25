package manipulation.strategies.parts.mobileManipulationNew.farNavigation;

import java.util.Observable;
import java.util.Observer;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.PTZPosition;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.itemMemory.ItemMemory.ReachingStatus;
import manipulation.strategies.MobileManipulationNew;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to approach an object from the distance and to reach a
 * position that the robot is able to recognize the object
 * 
 * @author ttoenige
 * 
 */
public class FarApproach extends StrategyPart implements Observer {
	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the fare approach part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public FarApproach(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FAR_APPROACH);

		manipulator.getPanTiltConnector().setPoseDeg(
				new PTZPosition(0, getManipulator().getConfiguration()
						.getFixedtiltAngle()));
	}

	private void alwaysGoToPosition() {

		try {
			getManipulator().getBaseConnector().goTo(

					((ViewPoint) getManipulator().getItemMemory()
							.getFirstGraspItem().getAttribute(
									PropertyName.BEST_VIEW_POINT))
							.getPosition());

		} catch (ExternalMemoryException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		} catch (InternalMemoryException e) {
			logger.warn(e);
		} catch (ItemException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		} catch (NullPointerException e) {
			// TODO ordentlichen Fehler schmeißen
			logger.error(e);
			try {
				getManipulator().getItemMemory().deleteItem(
						getManipulator().getItemMemory().getFirstGraspItem());
			} catch (InternalMemoryException e1) {
				logger.error(e1);
			}
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute " + this.getClass());
		getManipulator().getItemMemory().addObserver(this);

		alwaysGoToPosition();

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

	boolean firstTime = true;

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void update(Observable observable, Object arg) {
		if (observable instanceof ItemMemory) {
			if (arg instanceof Vector3D) {
				logger.error("Item changed in memory");
				alwaysGoToPosition();

				((MobileManipulationNew) getGlobalStrategy())
						.setCurrentTarget((Vector3D) arg);
			}

			if (arg instanceof ReachingStatus) {
				logger.debug("Reaching status changed in memory to " + arg);
				ReachingStatus currentStatus = ((ReachingStatus) arg);

				switch (currentStatus) {
				case VP_REACHED:
					logger.error("reached");
					setNextPartName(PartName.FAR_RECOGNIZE);
					synchronized (this) {
						notifyAll();
					}

					break;
				case VP_NOT_REACHABLE:
					logger.error("not reachable");
					try {
						getManipulator().getItemMemory().removeViewPoint(
								getManipulator().getItemMemory()
										.getFirstGraspItem(),
								(ViewPoint) getManipulator().getItemMemory()
										.getFirstGraspItem().getAttribute(
												PropertyName.BEST_VIEW_POINT),
								getManipulator());
						alwaysGoToPosition();
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
					logger.error("Try again");
					alwaysGoToPosition();
					break;
				default:
					break;
				}

			}
		}

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

}
