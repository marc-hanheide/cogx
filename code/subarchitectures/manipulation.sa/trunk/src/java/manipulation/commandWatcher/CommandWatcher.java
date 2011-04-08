package manipulation.commandWatcher;

import java.util.Observable;

import org.apache.log4j.Logger;

import manipulation.slice.ManipulationCommand;

public class CommandWatcher extends Observable {
	private Logger logger = Logger.getLogger(this.getClass());

	public enum ArmReachingStatus {
		REACHED, NOT_REACHED
	}

	public enum NavCommandName {
		PUT_DOWN,
	}

	public void armReachingStatusChange(boolean newReachingStatus) {
		setChanged();

		if (newReachingStatus) {
			notifyObservers(ArmReachingStatus.REACHED);
		} else {
			notifyObservers(ArmReachingStatus.NOT_REACHED);
		}

	}

	public void newCommand(ManipulationCommand cmd) {
		setChanged();
		notifyObservers(cmd);
	}

}
