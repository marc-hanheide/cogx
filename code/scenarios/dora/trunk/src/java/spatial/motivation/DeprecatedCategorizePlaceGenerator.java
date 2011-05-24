/**
 * 
 */
package spatial.motivation;

import motivation.components.generators.AbstractMotiveGenerator;
import motivation.factories.MotiveFactory;
import motivation.slice.CategorizePlaceMotive;
import motivation.slice.Motive;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 * @deprecated
 */
public class DeprecatedCategorizePlaceGenerator extends AbstractMotiveGenerator {

	/* (non-Javadoc)
	 * @see motivation.generators.Generator#updateMotive(cast.cdl.WorkingMemoryAddress, cast.cdl.WorkingMemoryAddress)
	 */
	@Override
	protected boolean checkMotive(Motive motive) throws CASTException {
			Place source = getMemoryEntry(motive.referenceEntry, Place.class);
			
			// if it is a yet unexplored one...			
			log("there is a place to be checked, created " + Long.toString(motive.created.s-getCASTTime().s) + " seconds ago");

			if (source.status == PlaceStatus.TRUEPLACE) {
				log("  it's a true place, so it should be considered as a motive to be categorized");
				write(motive);
				return true;
			}
			else {
				log("  turn out this place is not a trueplace, so it should be no motive then");
				remove(motive);
			}
		return false;
	}

	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		WorkingMemoryChangeReceiver receiver = new WorkingMemoryChangeReceiver() {
			
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				debug(CASTUtils.toString(_wmc));
				// create a new motive from this node...
				CategorizePlaceMotive newMotive = MotiveFactory.createCategorizePlaceMotive(_wmc.address);
				Place p;
				try {
					p = getMemoryEntry(_wmc.address, Place.class);
					newMotive.placeID=p.id;
				} catch (CASTException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} 
				scheduleCheckMotive(newMotive);
				
			}
		};
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), receiver);
	}

	/* (non-Javadoc)
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		// TODO Auto-generated method stub
		super.stop();
	}
	
	

}
