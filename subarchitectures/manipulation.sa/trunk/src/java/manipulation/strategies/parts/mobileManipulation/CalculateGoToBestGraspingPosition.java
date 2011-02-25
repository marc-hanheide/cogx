package manipulation.strategies.parts.mobileManipulation;

import java.util.Observable;
import java.util.Observer;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ViewPointException;
import manipulation.core.share.types.Region;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.itemMemory.ItemMemory.ReachingStatus;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

public class CalculateGoToBestGraspingPosition extends StrategyPart implements
		Observer {

	private Logger logger = Logger.getLogger(this.getClass());
	
	private ViewPoint bestPoint;

	/**
	 * constructor for the virtual fine approach part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public CalculateGoToBestGraspingPosition(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.CALCULATE_GOTO_BEST_GRASPING_POINT);

	}

	private void removeVPandGoOn() {
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

	public void approach() {
		try {

			bestPoint = getManipulator().getSimulationConnector()
					.getBestGraspingBasePoint();

			logger.error("Best grasping point in front: " + bestPoint);
			logger.error("Go to best grasping point in front");

			getManipulator().getBaseConnector().goTo(bestPoint.getPosition());

		} catch (ViewPointException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		} catch (ExternalMemoryException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		}

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);
			}
		}

		changeToNextPart();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute - calculate goto best grasping position - ");
		getManipulator().getItemMemory().addObserver(this);
		try {
			getManipulator().getSimulationConnector()
					.updateBestGraspingBasePoints(
							getManipulator().getBaseConnector()
									.getCurrentPosition(),
							getManipulator().getItemMemory()
									.getFirstGraspItem());

		} catch (ItemException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		} catch (InternalMemoryException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		} catch (ExternalMemoryException e) {
			logger.error(e);
			setNextPartName(PartName.GO_TO_START_POSITION);
			changeToNextPart();
		}

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
			if (arg instanceof Item) {
				logger.error("Item changed in memory");
			}

			if (arg instanceof Region) {
				logger.error("Region changed");
			}

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

					if (getManipulator().getSimulationConnector()
							.removeGraspingBasePoint(bestPoint)) {
						logger.error("another try");
						approach();
					} else {
						logger.error("No more Grasping point -> go on");
						removeVPandGoOn();
					}
					break;
				default:
					break;
				}

			}
		}

	}
}