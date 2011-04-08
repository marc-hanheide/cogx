package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.core.share.Manipulator;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.LinearBaseMovementApproachCommand;
import manipulation.slice.LinearGraspApproachCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateGraspCommand;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to reach a position in front of an object with the arm
 * 
 * @author ttoenige
 * 
 */
public class FarArmMovementCommandPart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	public FarArmMovementCommandPart(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FAR_ARM_MOVEMENT_COMMAND_PART);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		((CogXRunner) getManipulator().getRunner()).getWatcher().addObserver(
				this);

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
		((CogXRunner) getManipulator().getRunner()).getWatcher()
				.deleteObserver(this);

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));
	}

	/**
	 * 
	 * {@inheritDoc}
	 */
	@Override
	public void update(Observable observable, Object arg) {
		if (observable instanceof CommandWatcher) {
			if (arg instanceof FarArmMovementCommand) {
				logger.info("far arm movement command");
			} else if (arg instanceof PutDownCommand) {
				logger.info("put down command");
			} else if (arg instanceof LinearGraspApproachCommand) {
				logger.info("linear grasp approach command");
			} else if (arg instanceof SimulateGraspCommand) {
				logger.info("simulate grasp command");
			} else if (arg instanceof LinearBaseMovementApproachCommand) {
				logger.info("linear base movement approach command");
			}
		}
	}
}
