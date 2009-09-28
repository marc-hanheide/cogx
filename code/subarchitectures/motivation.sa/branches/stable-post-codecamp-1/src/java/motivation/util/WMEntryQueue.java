package motivation.util;

import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.SynchronousQueue;

import Ice.Object;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.Pair;

public class WMEntryQueue extends
		LinkedBlockingQueue<WMEntryQueue.QueueElement> implements
		WorkingMemoryChangeReceiver {

	public class QueueElement {
		/**
		 * @param addr
		 * @param entry
		 */
		QueueElement(WorkingMemoryChange wmc, Object entry) {
			this.wmc = wmc;
			this.entry = entry;
		}

		WorkingMemoryChange wmc;
		Ice.Object entry;

		/**
		 * @return the address
		 */
		public WorkingMemoryChange getEvent() {
			return wmc;
		}

		/**
		 * @return the entry
		 */
		public Ice.Object getEntry() {
			return entry;
		}

	}

	/**
	 * @param component
	 *            the component that is used to access the memory
	 */
	public WMEntryQueue(ManagedComponent component) {
		super();
		this.component = component;
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = -7628018827365709394L;

	protected ManagedComponent component;

	@Override
	public synchronized void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		try {
			if (wmc.operation != WorkingMemoryOperation.DELETE) {
				Ice.Object o = component.getMemoryEntry(wmc.address,
						Ice.Object.class);
				QueueElement qe = new QueueElement(wmc, o);
				this.put(qe);

			} else {
				QueueElement qe = new QueueElement(wmc, null);
				this.put(qe);
			}
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
