package manipulation.muster.strategies.parts.mobileManipulation.grasp;

import java.util.Observable;
import java.util.Observer;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.itemMemory.ItemMemory;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;
import manipulation.slice.ManipulationCommand;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to move the arm forward as long as nothing is between the
 * fingers of the gripper
 * 
 * @author ttoenige
 * 
 */
public class FineGrasping extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the fine grasp part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public FineGrasping(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FINE_GRASPING);

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		getManipulator().getItemMemory().addObserver(this);

		getManipulator().getArmConnector().reachFine();

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);
			}
		}

		getManipulator().getArmConnector().closeGripper(0);

		getManipulator().getArmConnector().goHome();

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

	@Override
	public void update(Observable observable, Object arg) {
		if (observable instanceof ItemMemory) {
			if (arg instanceof ManipulationCommand) {
				synchronized (this) {
					notifyAll();
				}
			}
		}

	}
}