/**
 * 
 */
package castutils.experimentation;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import motivation.slice.Motive;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.logging.ComponentLogger;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ChangeHandler;


/**
 * @author marc
 * 
 */
public class MotiveStatistics extends ManagedComponent implements
		ChangeHandler<Motive> {

	private ComponentLogger logger;

	/**
	 * @param motives
	 */
	public MotiveStatistics() {
		super();
		this.motives = WMView.create(this, Motive.class);
	}

	WMView<Motive> motives;
	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		logger = getLogger();
		try {
			motives.registerHandler(this);
			motives.start();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public synchronized void entryChanged(
			Map<WorkingMemoryAddress, Motive> map, WorkingMemoryChange wmc,
			Motive newEntry, Motive oldEntry) throws CASTException {
		Map<Class<? extends Motive>, Integer> counter = new HashMap<Class<? extends Motive>, Integer>();

		for (Motive motive : map.values()) {
			Integer oldValue = counter.get(motive.getClass());
			if (oldValue == null)
				oldValue = new Integer(0);
			counter.put(motive.getClass(), oldValue + 1);
		}
		for (Entry<Class<? extends Motive>, Integer> entry : counter.entrySet()) {
			logger.info(entry.getKey().getSimpleName().toUpperCase() + ": "
					+ entry.getValue());
			
		}

		// if there are now more motives we quit!
		if (map.size() == 0) {
		} else {
		}
		notifyAll();
	}

//	/*
//	 * (non-Javadoc)
//	 * 
//	 * @see cast.core.CASTComponent#runComponent()
//	 */
//	@Override
//	protected synchronized void runComponent() {
//		while (isRunning()) {
//			try {
//				wait();
//				if (shutdown) {
//					// wait 5 secs to make sure we really want to shut down
//					// (another motive might have turned up in between)
//					Thread.sleep(5000);
//					if (shutdown) {
//						logger
//								.error("shutting down whole CAST due to shutdown in "
//										+ MotiveStatistics.class
//												.getCanonicalName());
//						System.exit(1);
//					}
//				}
//			} catch (InterruptedException e) {
//				logger.error("exception: ", e);
//			}
//		}
//	}

}
