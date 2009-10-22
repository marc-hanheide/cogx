/**
 * 
 */
package coma.motivation;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import SpatialData.Place;

import motivation.components.generators.AbstractMotiveGenerator;
import motivation.factories.MotiveFactory;
import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.util.facades.SpatialFacade;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

import comadata.ComaRoom;

/**
 * create Motive for yet uncategorized rooms
 * 
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

			// we assume here, that a room initially has two concepts:
			// Portion_of_Space and PhysicalRoom; but we are trying to obtain
			// more
			
			if (source.concepts.length <= 2) {
				CategorizeRoomMotive crm = (CategorizeRoomMotive) motive;
				log("  nothing's known about it, so it should be considered as a motive");
				// The more places are contained the more information we get
				// from this room!
				crm.informationGain = 1.0-(1.0/source.containedPlaceIds.length);
				Place currentPlace = SpatialFacade.get(this).getPlace();
				double estimated_costs=0.0;
				for (long p : source.containedPlaceIds) {
					estimated_costs+=SpatialFacade.get(this).queryCosts(currentPlace.id, p);
				}
				if (source.containedPlaceIds.length>0)
					crm.costs = (float) (estimated_costs / source.containedPlaceIds.length);
				else
					crm.costs = (float) 0.0;
				
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
		} catch (CASTException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
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
		super.start();
		try {
			SpatialFacade.get(this).registerPlaceChangedCallback(
					new SpatialFacade.PlaceChangedHandler() {

						@Override
						public synchronized void update(Place p) {
							for (Ice.ObjectImpl m : motives.getMapByType(
									CategorizeRoomMotive.class).values())
								scheduleCheckMotive((Motive) m);

						}
					});
		} catch (CASTException e1) {
			println("exception when registering placeChangedCallbacks");
			e1.printStackTrace();
		}

		log("Starting up...");
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
