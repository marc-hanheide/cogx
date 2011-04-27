package manipulation.muster.strategies.parts.mobileManipulation.grasp;

import java.util.Observable;
import java.util.Observer;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.itemMemory.ItemMemory;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;
import manipulation.slice.ManipulationCommand;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to reach a position in front of an object with the arm
 * 
 * @author ttoenige
 * 
 */
public class FarGrasping extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	private Vector3D currentGoalPosition = new Vector3D(Double.MAX_VALUE,
			Double.MAX_VALUE, Double.MAX_VALUE);

	private Vector3D lastnewItemPosition;

	private Vector3D newItemPosition = new Vector3D(0, 0, 0);

	int notMovingcounter = 0;

	/**
	 * constructor for the grasping part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public FarGrasping(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FAR_GRASPING);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		getManipulator().getItemMemory().addObserver(this);

		getManipulator().getArmConnector().reach();

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);
			}
		}

		setNextPartName(PartName.FINE_GRASPING);
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
	 * 
	 * {@inheritDoc}
	 */
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
