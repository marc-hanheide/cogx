/**
 * 
 */
package motivation.util;

import java.util.Map;

import motivation.slice.Motive;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.CASTTimeUtil;

/**
 * @author marc
 * 
 */
public class WMMotiveDisplay extends ManagedComponent {
	WMMotiveView motives;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		motives = WMMotiveView.create(this);
		motives.setHandler(new WMMotiveView.ChangeHandler<Motive>() {
			@Override
			public void entryChanged(
					Map<WorkingMemoryAddress, Motive> map,
					WorkingMemoryChange wmc, Motive o,
					Motive old) {
				Motive motive =  o;
				println("a motive has been changed in set; WMaddress="
						+ motive.thisEntry.id);
				println("  type:      " + motive.getClass().getSimpleName());
				println("  operation: " + wmc.operation.name());
				println("  status:    " + motive.status.name());
				println("  age:       "
						+ Long.toString(CASTTimeUtil.diff(getCASTTime(),
								motive.created)));
			}

		});

		// start the motive listener...
		try {
			motives.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}

	}

}
