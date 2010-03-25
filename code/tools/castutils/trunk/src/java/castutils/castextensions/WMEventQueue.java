package castutils.castextensions;

import java.util.concurrent.LinkedBlockingQueue;

import cast.CASTException;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;

public class WMEventQueue extends LinkedBlockingQueue<WorkingMemoryChange>
		implements WorkingMemoryChangeReceiver {

	/**
	 * @param arg0
	 */
	public WMEventQueue() {
		super();
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = -7628018827365709394L;

	@Override
	public void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		System.out.println("WMEventQueue: received change");
		try {
			this.put(wmc);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.out.println("WMEventQueue: received change");
	}

}
