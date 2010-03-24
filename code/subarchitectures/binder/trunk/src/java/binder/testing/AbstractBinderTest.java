package binder.testing;

import cast.architecture.ManagedComponent;
import java.util.Map;


/**
 * Abstract class for testing facilities dedicated to the binder
 * 
 * @author Pierre Lison (plison@dfki.de)
 * @version 24.03.2010
 * @started 24.03.2010
 * 
 */
public abstract class AbstractBinderTest extends ManagedComponent {

	
	// System time at test initialization
	long initTime;
	
	// System time once test is finished
	long endTime;
	

	// default timeout value (in seconds)
	public static final int DEFAULT_TIMEOUT = 10;	
	
	// actual time out value
	private int timeout = DEFAULT_TIMEOUT;
	
	
	/**
	 * Configuration method.  The component can be parametrized in the CAST configuration
	 * file to specify a particular timeout value, different from the default one.
	 * 
	 * @config set of <parameter,value> pairs
	 */
	@Override
	public void configure(Map<String, String> config) {
		if (config.containsKey("--timeout")) {
			String timeoutStr = config.get("--timeout");
			try {
				timeout = Integer.parseInt(timeoutStr);
			}
			catch (NumberFormatException e) {
				log("Warning, wrong timeout format, switching to default timeout");
			}
		}
	}
	
	
	
	/**
	 * Execution method, automatically triggered after component is initialised
	 */
	@Override
	public void runComponent() {
				
		
		log("Now running test: " + getTestName());
		log("Description: " + getTestDescription() + "\n");
		
		// first, start the timer
		startTimer();
		
		// start the test
		startTest();
			
		boolean finished = false;
		while (!finished) {
			
			if (isTestFinished()) {
				
				// check if test is successful
				if (isTestSuccessful()) {
					log("Test successful!");
				}
				
				// if test failed
				else {
					log("Test failed...");
				}	
				endTimer();
				finished = true;
			}
			
			// if we are still below the time out value
			else if (getCurrentTime() < getTimeoutValue() * 1000) {
				debug("Test not finished yet, waiting");
				sleepComponent(50);		
			}
			
			// if we reached the timeout
			else {
				log("Test still not finished after timeout (" + (getTimeoutValue()*1000) + " ms), aborting");	
				endTimer();
				finished = true;
			}
		}
		
		log("Total running time: "+ getTotalTimeInSeconds());
	}
	
	/**
	 * Start the timer
	 */
	public void startTimer() {
		initTime = System.currentTimeMillis();
	}
	
	/**
	 * End the timer
	 */
	public void endTimer() {
		endTime = System.currentTimeMillis();
	}
	
	/**
	 * Get the current running time (i.e. current system time - system time at initialisation)
	 * 
	 * @return current running time (in milliseconds)
	 */
	public long getCurrentTime() {
		return initTime - System.currentTimeMillis();
	}
	

	/**
	 * Get the (rounded) total running time, in seconds
	 * 
	 * @return running time
	 */
	public double getTotalTimeInSeconds() {
		double totalTimeSecs = (endTime - initTime) / 1000.0;
		return Math.round(totalTimeSecs*100.0) / 100.0;
	}
	
	
	/**
	 * Return timeout value (in seconds)
	 * @return timeout value
	 */
	public long getTimeoutValue() {
		return timeout;
	}
	
	// =========================================================
	// ABSTRACT METHODS
	// =========================================================
	
	
	/**
	 * Return short name for the test
	 * 
	 * @return string containing test name
	 */
	public abstract String getTestName();
	
	
	/**
	 * Return short (one sentence) description of the test
	 * 
	 * @return string containing test description
	 */
	public abstract String getTestDescription();


	
	/**
	 * Start the test
	 */
	public abstract void startTest();

	
	/**
	 * Returns true if test is finished, false if it isn't 
	 * 
	 * @return true if finished, false otherwise
	 */
	public abstract boolean isTestFinished();
	
	
	/**
	 * Returns true if test is finished and successful, false if not
	 * 
	 * @return true if finished ^ successful, false in other cases
	 */
	public abstract boolean isTestSuccessful();

	
	/**
	 * Returns a string containing the reason for the test failure, if the
	 * test indeed failed.  If the test was successful, returns an empty string
	 
	 * @return string containing reason for test failure
	 */
	public abstract String getReasonForFailure();
	
	
}
