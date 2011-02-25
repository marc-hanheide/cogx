package manipulation.strategies.parts.mobileManipulationNew.farNavigation;

import java.util.Observable;
import java.util.Observer;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.itemMemory.ItemMemory;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.itemMemory.ItemMemory.ReachingStatus;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;
import manipulation.strategies.parts.StrategyPart.PartName;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to go to the home position of the robot (only base
 * movement)
 * 
 * @author ttoenige
 * 
 */
public class GoToStartPosition extends StrategyPart implements Observer {
	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the go home part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public GoToStartPosition(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.GO_TO_START_POSITION);
	}

	private void goHome() {
		try {
			getManipulator().getBaseConnector().goTo(
					new BasePositionData(0, 0, 0));
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		getManipulator().getItemMemory().addObserver(this);

		logger.debug("execute: " + this.getClass());
		goHome();

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
			if (arg instanceof Vector3D) {
				logger.error("Item changed in memory");
				setNextPartName(PartName.FAR_APPROACH);
				synchronized (this) {
					notifyAll();
				}

			}

		}
	}

}
