package motivation.util;

import java.util.Map;
import java.util.concurrent.LinkedBlockingQueue;

import motivation.slice.Motive;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;

public class WMMotiveEventQueue extends LinkedBlockingQueue<WMMotiveEventQueue.MotiveEvent> implements WMView.ChangeHandler<Motive>{

	public class MotiveEvent {
		public Map<WorkingMemoryAddress, Motive> map;
		public WorkingMemoryChange wmc; 
		public Motive newMotive; 
		public Motive oldMotive;
	}
	
	/**
	 * 
	 */
	private static final long serialVersionUID = -7628018827365709394L;


	@Override
	public void entryChanged(Map<WorkingMemoryAddress, Motive> map,
			WorkingMemoryChange wmc, Motive newMotive, Motive oldMotive) {
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
