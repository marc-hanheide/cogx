package manipulation.muster.strategies.parts.mobileManipulation.grasp;

import java.util.HashMap;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.exceptions.ExternalMemoryException;
import manipulation.muster.core.share.exceptions.InternalMemoryException;
import manipulation.muster.core.share.exceptions.ItemException;
import manipulation.muster.core.share.exceptions.ManipulatorException;
import manipulation.muster.core.share.types.ArmError;
import manipulation.muster.core.share.types.Matrix;
import manipulation.muster.core.share.types.SensorData.SensorPosition;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.itemMemory.Item.PropertyName;
import manipulation.muster.math.MathOperation;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to move the arm forward as long as nothing is between the
 * fingers of the gripper
 * 
 * @author ttoenige
 * 
 */
public class FineGrasping extends StrategyPart {

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


		logger.debug("we go on!");

		changeToNextPart();

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {
		logger.error("Aufräumen fertig - weiter gehts");

		try {
			getManipulator().getCamConnector().resetTracker();
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}
}