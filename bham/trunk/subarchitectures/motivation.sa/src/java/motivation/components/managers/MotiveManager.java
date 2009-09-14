/**
 * 
 */
package motivation.components.managers;

import motivation.slice.Motive;
import motivation.util.WMMotiveSet;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public abstract class MotiveManager extends ManagedComponent {

	Class<? extends Motive> specificType;
	WMMotiveSet motives;

	/**
	 * @param specificType
	 */
	protected MotiveManager(Class<? extends Motive> specificType) {
		super();
		this.specificType = specificType;
		motives = WMMotiveSet.create(this, specificType);
	}

	@Override
	protected void start() {
	
		log("MotiveManager starting up...");
		super.start();

		motives.setHandler(new WMMotiveSet.ChangeHandler() {

			@Override
			public void motiveChanged(WorkingMemoryChange wmc, Motive motive) {
				log("motive has been changed in map: Status is "
						+ motive.status.name());
				if ((wmc.operation == WorkingMemoryOperation.ADD)
						|| (wmc.operation == WorkingMemoryOperation.OVERWRITE)) {
					switch (motive.status) {
					case UNSURFACED:
						retractMotive(motive);
						break;
					case SURFACED:
						manageMotive(motive);
						break;
					}
				}
				else if(wmc.operation == WorkingMemoryOperation.DELETE) {
					retractMotive(motive);
				}
			}

		});

		// start the motive listener...
		motives.start();
	}

	protected abstract void manageMotive(Motive motive);

	protected abstract void retractMotive(Motive motive);
}
