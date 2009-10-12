/**
 * 
 */
package coma.motivation;

import motivation.components.generators.AbstractMotiveGenerator;
import motivation.factories.MotiveFactory;
import motivation.slice.CategorizeRoomMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

import comadata.ComaRoom;

/** create Motive for yet uncategorized rooms
 * @author marc
 * 
 */
public class CategorizeRoomGenerator extends AbstractMotiveGenerator {

	public CategorizeRoomGenerator() {
		super();
	}

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
			ComaRoom source = getMemoryEntry(motive.referenceEntry,
					ComaRoom.class);

			// if it is a yet unexplored one...
			log("there is a RoomCategorizeMotive to be checked, created "
					+ Long.toString(motive.created.s - getCASTTime().s)
					+ " seconds ago");

			if (source.concepts.length == 0) {
				CategorizeRoomMotive crm = (CategorizeRoomMotive) motive;
				log("  nothing's known about it, so it should be considered as a motive");
				crm.costs=1;
				// The more places are contained the more information we get from this room!
				crm.informationGain=source.containedPlaceIds.length;
				crm.roomId = source.roomId;
				write(crm);
				return true;
			} else {
				log("  turns out this room is categorized already, so it should be no motive then");
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

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ComaRoom.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						debug(CASTUtils.toString(_wmc));
						// create a new motive from this node...
						CategorizeRoomMotive newMotive = MotiveFactory
								.createCategorizeRoomMotive(_wmc.address);

						// ComaRoom p;
						// try {
						// p = getMemoryEntry(_wmc.address, ComaRoom.class);
						// // initialize fields of motive here
						// } catch (CASTException e) {
						// e.printStackTrace();
						// }
						checkMotive(newMotive);
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
