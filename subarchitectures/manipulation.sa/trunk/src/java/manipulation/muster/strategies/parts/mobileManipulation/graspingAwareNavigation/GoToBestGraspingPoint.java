package manipulation.muster.strategies.parts.mobileManipulation.graspingAwareNavigation;

import java.util.Observable;
import java.util.Observer;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.itemMemory.ItemMemory;
import manipulation.muster.itemMemory.ItemMemory.ReachingStatus;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;

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
		logger.error("Go to best grasping point in front");

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

					//TODO was tun
					
					break;
				default:
					break;
				}

			}
		}

	}
}