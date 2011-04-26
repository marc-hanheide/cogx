package manipulation.muster.strategies.parts.mobileManipulation.grasp;

import java.util.Observable;
import java.util.Observer;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.exceptions.ExternalMemoryException;
import manipulation.muster.core.share.exceptions.InternalMemoryException;
import manipulation.muster.core.share.exceptions.ItemException;
import manipulation.muster.core.share.exceptions.ManipulatorException;
import manipulation.muster.core.share.types.ArmError;
import manipulation.muster.core.share.types.Matrix;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.itemMemory.Item;
import manipulation.muster.itemMemory.Item.PropertyName;
import manipulation.muster.itemMemory.ItemMemory;
import manipulation.muster.math.MathOperation;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;

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



	private void stopTracking() {
		try {
			getManipulator().getCamConnector().resetTracker();
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
	 * 
	 * {@inheritDoc}
	 */
	@Override
	public void update(Observable observable, Object arg) {
		if (observable instanceof ItemMemory) {
			if (arg instanceof Vector3D) {
				try {
					Item newItem = null;
					try {
						newItem = ((ItemMemory) observable).getFirstGraspItem();
					} catch (InternalMemoryException e) {
						logger.error(e);
					}
					lastnewItemPosition = newItemPosition;

					newItemPosition = ((Vector3D) newItem
							.getAttribute(PropertyName.WORLD_POSITION));

					if (MathOperation.getDistance(currentGoalPosition,
							newItemPosition) > 0.05) {
						if (!(MathOperation.getDistance(lastnewItemPosition,
								newItemPosition) > 0.002)) {
							currentGoalPosition = newItemPosition;
							logger.error("not moving -> grasping");

						}
					}
				} catch (ItemException e1) {
					logger.error(e1);
					stopTracking();
					setNextPartName(PartName.FAR_APPROACH);
					synchronized (this) {
						notifyAll();
					}
				}
			}
		}
	}
}
