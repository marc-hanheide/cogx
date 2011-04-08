package manipulation.commandWatcher;

import java.util.Observable;

import org.apache.log4j.Logger;

import cast.cdl.WorkingMemoryAddress;

import manipulation.slice.ManipulationCommand;

public class CommandWatcher extends Observable {
	private Logger logger = Logger.getLogger(this.getClass());

	public enum ArmReachingStatus {
		REACHED
	}

	public WorkingMemoryAddress currentCommandAddress;

	/**
	 * @return the currentCommandAddress
	 */
	public WorkingMemoryAddress getCurrentCommandAddress() {
		return currentCommandAddress;
	}

	/**
	 * @param currentCommandAddress
	 *            the currentCommandAddress to set
	 */
	public void setCurrentCommandAddress(
			WorkingMemoryAddress currentCommandAddress) {
		this.currentCommandAddress = currentCommandAddress;
	}

	public void newCommand(ManipulationCommand cmd) {
		setChanged();
		notifyObservers(cmd);
	}

	public void posReached() {
		setChanged();
		notifyObservers(ArmReachingStatus.REACHED);
	}
}
