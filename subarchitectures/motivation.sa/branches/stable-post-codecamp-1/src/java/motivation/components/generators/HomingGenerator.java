/**
 * 
 */
package motivation.components.generators;

import motivation.factories.MotiveFactory;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class HomingGenerator extends AbstractMotiveGenerator {

	Place homeBase = null;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * motivation.generators.Generator#updateMotive(cast.cdl.WorkingMemoryAddress
	 * , cast.cdl.WorkingMemoryAddress)
	 */
	@Override
	protected boolean checkMotive(Motive motive) {
		try {
			Place source = getMemoryEntry(motive.referenceEntry, Place.class);

			if (source.status == PlaceStatus.TRUEPLACE) {
				log("  we have a first true place and take it as the home base");
				homeBase=source;
				write(motive);
				return true;
			}
		} catch (CASTException e) {
			e.printStackTrace();
		} 
		return false;
	}

	// (= position(agent) position_id)

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				if (homeBase == null) { // if home base is not set yet
					HomingMotive newMotive = MotiveFactory
							.createHomingMotive(_wmc.address);
					Place p;
					try {

						p = getMemoryEntry(_wmc.address, Place.class);
						newMotive.homePlaceID = p.id;
					} catch (CASTException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					checkMotive(newMotive);
				} else { // if we have a home base already, remove the listener, we are happy!
					try {
						removeChangeFilter(this);
					} catch (SubarchitectureComponentException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		});
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		// TODO Auto-generated method stub
		super.stop();
	}

}
