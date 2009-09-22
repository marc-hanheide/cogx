package motivation.util;

import java.util.concurrent.SynchronousQueue;

import cast.CASTException;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;

public class WMEventQueue extends SynchronousQueue<WorkingMemoryChange> implements WorkingMemoryChangeReceiver{

	/**
	 * 
	 */
	private static final long serialVersionUID = -7628018827365709394L;

	@Override
	public void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		this.add(wmc);
	}
	
}
