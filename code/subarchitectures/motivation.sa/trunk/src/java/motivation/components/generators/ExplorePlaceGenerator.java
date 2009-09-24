/**
 * 
 */
package motivation.components.generators;

import motivation.factories.MotiveFactory;
import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 *
 */
public class ExplorePlaceGenerator extends AbstractMotiveGenerator {

	/* (non-Javadoc)
	 * @see motivation.generators.Generator#updateMotive(cast.cdl.WorkingMemoryAddress, cast.cdl.WorkingMemoryAddress)
	 */
	@Override
	protected boolean checkMotive(Motive motive) {
		try {
			Place source = getMemoryEntry(motive.referenceEntry, Place.class);
			
			// if it is a yet unexplored one...			
			log("there is a place to be checked, created " + Long.toString(motive.created.s-getCASTTime().s) + " seconds ago");

			if (source.status == PlaceStatus.PLACEHOLDER) {
				log("  it's a placeholder, so it should be considered as a motive");
				write(motive);
				return true;
			}
			else {
				log("  turn out this place is not a placeholder, so, it should be no motive then");
				remove(motive);
			}
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			e.printStackTrace();
		} catch (PermissionException e) {
			e.printStackTrace();
		}
		return false;
	}

	/* (non-Javadoc)
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
				ExploreMotive newMotive = MotiveFactory.createExploreMotive(_wmc.address);
				Place p;
				try {
					p = getMemoryEntry(_wmc.address, Place.class);
					newMotive.placeID=p.id;
				} catch (CASTException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} 
				checkMotive(newMotive);
			}
		});
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
