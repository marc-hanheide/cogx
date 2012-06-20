package castutils.castextensions;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import org.apache.log4j.Logger;

import cast.CASTException;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTComponent;

/**
 * This realizes a {@link BlockingQueue} that can be used as a
 * {@link WorkingMemoryChangeReceiver} to serialize events. Often used to avoid
 * concurrency in a {@link CASTComponent} by taking entries off this queue in
 * runComponent().
 * 
 * @author marc
 * 
 */
public class WMEventQueue extends LinkedBlockingQueue<WorkingMemoryChange>
		implements WorkingMemoryChangeReceiver {

	private static final long serialVersionUID = -7628018827365709394L;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * cast.architecture.WorkingMemoryChangeReceiver#workingMemoryChanged(cast
	 * .cdl.WorkingMemoryChange)
	 */
	@Override
	public void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		try {
			this.put(wmc);
		} catch (InterruptedException e) {
			Logger.getLogger(WMEventQueue.class).equals(e);
			throw new CASTException("rethrown after an InterruptedException: "
					+ e.getMessage());
		}
	}

}
