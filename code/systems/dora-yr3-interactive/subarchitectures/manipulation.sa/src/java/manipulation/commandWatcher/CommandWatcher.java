package manipulation.commandWatcher;

import java.util.Observable;

import org.apache.log4j.Logger;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.cdl.WorkingMemoryAddress;

import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.ManipulationCommand;
import manipulation.slice.ManipulationExternalCommand;

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
	public WorkingMemoryAddress newCommandAddress = null;

	public WorkingMemoryAddress lastCommandAddress = null;

	/**
	 * gets the external memory address of the new command
	 * 
	 * @return newCommandAddress
	 */
	public WorkingMemoryAddress getNewCommandAddress() {
		return newCommandAddress;
	}

	public WorkingMemoryAddress getLastCommandAddress() {
		return lastCommandAddress;
	}

	/**
	 * sets the new command external memory address
	 * 
	 * @param newCommandAddressNew
	 *            the newCommandAddressNew to set
	 */
	public void setCurrentCommandAddress(
			WorkingMemoryAddress newCommandAddressNew) {
		this.lastCommandAddress = this.newCommandAddress;
		this.newCommandAddress = newCommandAddressNew;

		logger.error("Last Command Address: " + this.lastCommandAddress);
		logger.error("Current Command Address: " + this.newCommandAddress);

	}

	/**
	 * sets a new command and notify all observers
	 * 
	 * @param cmd
	 *            new command to set
	 */
	public void newCommand(ManipulationExternalCommand cmd) {
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
