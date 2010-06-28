/**
 * 
 */
package coma.motivation;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import motivation.components.generators.AbstractMotiveGenerator;
import motivation.factories.MotiveFactory;
import motivation.slice.CategorizeRoomMotive;
import motivation.slice.Motive;
import castutils.facades.BinderFacade;
import castutils.facades.SpatialFacade;
import SpatialData.Place;
import beliefmodels.autogen.beliefs.Belief;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
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


	private static final double EXP_NORM = 8.0;

	private static final int MIN_PLACES_PER_ROOM = 2;

	private static final float MAX_CATEGORIZE_COSTS = 10;

	private boolean blockRooms;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		if ((config.get("--blockrooms")) != null) {
			blockRooms = true;
			println("***** CAUTION: There are blocked rooms defined in "
					+ CategorizeRoomGenerator.class.getCanonicalName());
		} else
			blockRooms = false;
	}

	/**
	 * This is an array of roomIds to be blocked, i.e. no CategorizeRoomMotive
	 * are generated when this ComaRomm is created
	 */
	final private long[] blockedRoomIds = { 0 };

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
				computeAttributes(crm, source);
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

	/**
	 * compute the attributes of the motive
	 * 
	 * @param crm
	 * @param source
	 * @throws CASTException
	 * @throws InterruptedException
	 */
	protected void computeAttributes(CategorizeRoomMotive crm, ComaRoom source)
			throws CASTException, InterruptedException {
		final SpatialFacade sf = SpatialFacade.get(this);
		final BinderFacade binder = BinderFacade.get(this);

		int countRealPlaces = 0;
		int countPlaceHolders = 0;
		{ // find all place holders from binder
			Set<Place> placesInRoom = sf.getPlaces(source);
			Set<Long> placeHolderIds = new HashSet<Long>();
			Set<Long> truePlaceIds = new HashSet<Long>();
			log("number of places in room: " + placesInRoom.size());
			for (Place p : placesInRoom) {
				Belief pp = binder.findPlaceProxy(p);
				truePlaceIds.add(p.id);
				if (pp != null) {
					log("looking for related proxies");
					Map<WorkingMemoryAddress, Belief> related = binder
							.getRelatedProxies(pp, "connected");
					log("  found " + related.size()
							+ " related proxies for place " + p.id);
					for (Belief prx : related.values()) {
						Long id = new Long(binder.getPlaceIdFromProxy(prx));
						// if this connected place is not in the list of true
						// places add it as a placeholder
						if (!truePlaceIds.contains(id)) {
							placeHolderIds.add(id);
						}
					}

				} else {
					log("  could not find proxy for place_id " + p.id);
				}
			}
			countRealPlaces = truePlaceIds.size();
			countPlaceHolders = placeHolderIds.size();

			{ // some debugging logs
				log("countRealPlaces = " + countRealPlaces);
				log("countPlaceHolders = " + countPlaceHolders);
				String tmp = "  placesInRoom:";
				for (Long p : truePlaceIds)
					tmp = tmp + " " + p;
				log(tmp);
				tmp = "  placeHolder: ";
				for (Long p : placeHolderIds)
					tmp = tmp + " " + p;
				log(tmp);

			}
		}
		// log("  countRealPlaces = "+ countRealPlaces);
		Place currentPlace = sf.getPlace();
		// compute the mean costs to reach the places in the room
		double estimatedTravelCosts = 0.0;
//		for (long p : source.containedPlaceIds) {
//			estimatedTravelCosts += sf.queryCosts(currentPlace.id, p);
//		}
//		if (source.containedPlaceIds.length > 0)
//			crm.costs = (float) (estimatedTravelCosts / source.containedPlaceIds.length);
//		else
//			crm.costs = (float) 0.0;
		if (source.containedPlaceIds.length > 0) {
			estimatedTravelCosts = sf.queryCosts(currentPlace.id, source.containedPlaceIds[0]);
		} else {
			estimatedTravelCosts = Double.MAX_VALUE;
		}
		log("  estimatedTravelCosts = " + crm.costs);
		crm.costs = (float) estimatedTravelCosts;
		
		// the ratio between explored places and number of places
		double exploredRatio = (double) countPlaceHolders
				/ (double) (countPlaceHolders+countRealPlaces);
		log("  exploredRatio = " + exploredRatio);
		// we consider not fully explored rooms to be ways more expensive...
		float categorizeCostEstimate = (float) (MAX_CATEGORIZE_COSTS * exploredRatio); 
		crm.costs = (float) crm.costs + categorizeCostEstimate;
		log("  costs = " + crm.costs);

		// The more (real) places are contained the more information we get
		// from this room!
		crm.informationGain = 1 - (1. / Math
				.exp((((double) countRealPlaces) / EXP_NORM) * Math.log(2)));

//		if (source.containedPlaceIds.length<MIN_PLACES_PER_ROOM) {
//			crm.informationGain=0.0;
//		}
		
		log("  informationGain = " + crm.informationGain);

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		WorkingMemoryChangeReceiver checkAllReceiver = new WorkingMemoryChangeReceiver() {
			
			@Override
			public void workingMemoryChanged(WorkingMemoryChange wmc)
					throws CASTException {
				for (Ice.ObjectImpl m : motives.getMapByType(
						CategorizeRoomMotive.class).values())
					scheduleCheckMotive((Motive) m);
			}
		};
		// let any change to places trigger check of all room motives
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				Place.class, WorkingMemoryOperation.WILDCARD),
				checkAllReceiver);
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ComaRoom.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						debug(CASTUtils.toString(_wmc));
						// create a new motive from this node...
						CategorizeRoomMotive newMotive = MotiveFactory
								.createCategorizeRoomMotive(_wmc.address);

						ComaRoom p;

						try {
							p = getMemoryEntry(_wmc.address, ComaRoom.class);
							if (blockRooms) {
								for (long blockedRoom : blockedRoomIds) {
									if (p.roomId == blockedRoom) {
										println("ignore room " + p.roomId
												+ " as it is a blocked rooms");
										return;
									}
								}
							}
						} catch (CASTException e) {
							e.printStackTrace();
						}
						scheduleCheckMotive(newMotive);
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
