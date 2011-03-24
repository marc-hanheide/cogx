/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.beliefs.utils;

import java.util.concurrent.LinkedBlockingQueue;

import cast.CASTException;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.WorkingMemoryReaderComponent;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class BeliefEventQueue extends LinkedBlockingQueue<WorkingMemoryChange>
		implements WorkingMemoryChangeReceiver {

	/**
	 * @param component
	 */
	public BeliefEventQueue(WorkingMemoryReaderComponent component, String type) {
		super();
		this.component = component;
		this.type = type;
	}

	final WorkingMemoryReaderComponent component;
	final String type;
	
	/**
	 * 
	 */
	private static final long serialVersionUID = 7743911052770337782L;

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
			dBelief belief = component.getMemoryEntry(wmc.address, dBelief.class);
			if (type.equals(belief.type))
				this.put(wmc);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}