package manipulation.commandWatcher;

import java.util.Observable;

import org.apache.log4j.Logger;

import cast.cdl.WorkingMemoryAddress;

import manipulation.slice.ManipulationCommand;

/**
 * Watcher that takes care of the different commands which were added to the
 * external memory
 * 
 * @author Torben Toeniges
 * 
 */
public class CommandWatcher extends Observable {
	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * status of the current arm movement
	 * 
	 * @author Torben Toeniges
	 * 
	 */
	public enum ArmReachingStatus {
		REACHED
	}

	/**
	 * external memory address of the current command
	 */
	public WorkingMemoryAddress currentCommandAddress;

	/**
	 * gets the external memory address of the current command
	 * 
	 * @return the currentCommandAddress
	 */
	public WorkingMemoryAddress getCurrentCommandAddress() {
		return currentCommandAddress;
	}

	/**
	 * sets the current command external memory address
	 * 
	 * @param currentCommandAddress
	 *            the currentCommandAddress to set
	 */
	public void setCurrentCommandAddress(
			WorkingMemoryAddress currentCommandAddress) {
		this.currentCommandAddress = currentCommandAddress;
	}

	/**
	 * sets a new command and notify all observers
	 * 
	 * @param cmd
	 *            new command to set
	 */
	public void newCommand(ManipulationCommand cmd) {
		setChanged();
		notifyObservers(cmd);
	}

	/**
	 * Function which is called when a position is reached with the arm. Informs
	 * all observers about the position reaching
	 */
	public void posReached() {
		setChanged();
		notifyObservers(ArmReachingStatus.REACHED);
	}
}
