/**
 * 
 */
package motivation.util;

import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.TestMotive;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.CASTTimeUtil;

/**
 * @author marc
 *
 */
public class WMMotiveDisplay  extends ManagedComponent {
	WMMotiveSet motives;

	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		motives = WMMotiveSet.create(this);
		motives.setHandler(new WMMotiveSet.ChangeHandler() {
			@Override
			public void entryChanged(Map<WorkingMemoryAddress, Ice.ObjectImpl> map, WorkingMemoryChange wmc, Ice.ObjectImpl o, Ice.ObjectImpl old) {
				Motive motive = (Motive) o;
				println("a motive has been changed in set; WMaddress="+motive.thisEntry.id);
				println("  type:      " + motive.getClass().getSimpleName());
				if (motive.getClass().getSimpleName().equals("TestMotive")) {
					TestMotive m = (TestMotive) motive;
					println("    specific: "+m.value);
				}
				println("  operation: " + wmc.operation.name());
				println("  status:    " + motive.status.name());
				println("  age:       " + Long.toString(CASTTimeUtil.diff(getCASTTime(), motive.created)));
			}


		});

		// start the motive listener...
		motives.start();
		
	}
	
	
	
}
