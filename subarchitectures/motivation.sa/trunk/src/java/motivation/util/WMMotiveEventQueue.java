package motivation.util;

import java.util.Map;
import java.util.concurrent.LinkedBlockingQueue;

import Ice.ObjectImpl;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEntrySet;

public class WMMotiveEventQueue extends LinkedBlockingQueue<WMMotiveEventQueue.MotiveEvent> implements WMEntrySet.ChangeHandler{

	public class MotiveEvent {
		public Map<WorkingMemoryAddress, ObjectImpl> map;
		public WorkingMemoryChange wmc; 
		public ObjectImpl newMotive; 
		public ObjectImpl oldMotive;
	}
	
	/**
	 * 
	 */
	private static final long serialVersionUID = -7628018827365709394L;


	@Override
	public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
			WorkingMemoryChange wmc, ObjectImpl newMotive, ObjectImpl oldMotive) {
		MotiveEvent m = new MotiveEvent(); 
		try {
			m.map=map;
			m.wmc=wmc;
			m.newMotive=newMotive;
			m.oldMotive=oldMotive;
			this.put(m);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}		
	}
	
}
