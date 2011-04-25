package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import org.apache.log4j.Logger;

import manipulation.core.share.Manipulator;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

public class GetCurrentArmPosePart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());
	
	public GetCurrentArmPosePart(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.GET_CURRENT_ARM_POSE_PART);
	}

	@Override
	public void changeToNextPart() {
		getManipulator().getWatcher().deleteObserver(this);

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}

	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());
		
		getManipulator().getWatcher().addObserver(this);
	}

	@Override
	public void update(Observable arg0, Object arg1) {

	}
}
