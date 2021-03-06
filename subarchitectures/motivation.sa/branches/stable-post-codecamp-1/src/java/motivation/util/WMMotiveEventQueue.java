package motivation.util;

import java.util.Map;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.SynchronousQueue;

import motivation.slice.Motive;

import Ice.ObjectImpl;

import cast.CASTException;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;

public class WMMotiveEventQueue extends LinkedBlockingQueue<WMMotiveEventQueue.MotiveEvent> implements WMEntrySet.ChangeHandler{

	public class MotiveEvent {
		Map<WorkingMemoryAddress, ObjectImpl> map;
		WorkingMemoryChange wmc; ObjectImpl newMotive; ObjectImpl oldMotive;
	}
	
	/**
	 * 
	 */
	private static final long serialVersionUID = -7628018827365709394L;


	@Override
	public void motiveChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
			WorkingMemoryChange wmc, ObjectImpl newMotive, ObjectImpl oldMotive) {
		MotiveEvent m = new MotiveEvent(); 
		try {
			this.put(m);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
	}
	
}
