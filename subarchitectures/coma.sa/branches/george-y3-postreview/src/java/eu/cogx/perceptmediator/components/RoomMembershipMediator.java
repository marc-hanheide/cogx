/**
 * 
 */
package eu.cogx.perceptmediator.components;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import castutils.castextensions.WMContentWaiter;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMView;
import castutils.castextensions.WMContentWaiter.ContentMatchingFunction;

import comadata.ComaRoom;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.helpers.ComaRoomMatchingFunction;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

/**
 * @author marc
 * 
 */
public class RoomMembershipMediator extends ManagedComponent {

	public static final String ROOM_PROPERTY = "in-room";
	final Map<WorkingMemoryAddress, Set<WorkingMemoryAddress>> mapRoom2Places = new HashMap<WorkingMemoryAddress, Set<WorkingMemoryAddress>>();

	WMView<GroundedBelief> allBeliefs;
	final WMContentWaiter<GroundedBelief> waitingBeliefReader;
	final WMEventQueue entryQueue = new WMEventQueue();

	public RoomMembershipMediator() {
		this.allBeliefs = WMView.create(this, GroundedBelief.class);
		this.waitingBeliefReader = new WMContentWaiter<GroundedBelief>(
				this.allBeliefs);
	}

	/**
	 * this method tries to dereference a referred belief. It waits for the
	 * referred object to appear and blocks until then.
	 * 
	 * @param contentMatchingFunction
	 *            the matching function to be used
	 * @see WMContentWaiter
	 * @return the working memory address that corresponds
	 * @throws InterruptedException
	 */
	protected WorkingMemoryAddress getReferredBelief(
			ContentMatchingFunction<? super GroundedBelief> contentMatchingFunction)
			throws InterruptedException {
		debug("trying to find referred belief");
		while (true) {
			Entry<WorkingMemoryAddress, GroundedBelief> entry = waitingBeliefReader
					.read(contentMatchingFunction);
			if (entry != null) {
				debug("got it: " + entry.getKey().id);
				return entry.getKey();
			} else {
				getLogger().warn("waiting for belief VERY long now... keep on waiting");
			}
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		try {
			while (true) {
				try {
					final WorkingMemoryChange ev = entryQueue.take();
					log(RoomMembershipMediator.class.getName() + " change: "
							+ CASTUtils.toString(ev));
					switch (ev.operation) {
					case ADD: {
						ComaRoom room = getMemoryEntry(ev.address,
								ComaRoom.class);
						addPlaceProperties(room, ev.address);
						mapRoom2Places.put(ev.address,
								getContainedPlaceSet(room));
						break;
					}
					case OVERWRITE: {
						ComaRoom room = null;
						try {
							room = getMemoryEntry(ev.address, ComaRoom.class);
						} catch (DoesNotExistOnWMException e) {
							getLogger().warn("object already removed...");
						}
						if (room != null) {
							removeInvalidPlaceProperties(room, ev.address);
							addPlaceProperties(room, ev.address);

							mapRoom2Places.put(ev.address,
									getContainedPlaceSet(room));
						} else {
							removePlaceProperties(ev.address);
							mapRoom2Places.remove(ev.address);
						}
						break;
					}
					case DELETE: {
						removePlaceProperties(ev.address);
						mapRoom2Places.remove(ev.address);
						break;
					}
					}
				} catch (InterruptedException e) {
					logException(e);
				}
			}
		} catch (CASTException e) {
			logException(e);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(ComaRoom.class,
				WorkingMemoryOperation.ADD), entryQueue);
		addChangeFilter(ChangeFilterFactory.createTypeFilter(ComaRoom.class,
				WorkingMemoryOperation.OVERWRITE), entryQueue);
		addChangeFilter(ChangeFilterFactory.createTypeFilter(ComaRoom.class,
				WorkingMemoryOperation.DELETE), entryQueue);
		try {
			allBeliefs.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	private void addPlaceProperties(ComaRoom room, WorkingMemoryAddress roomAddr)
			throws CASTException, InterruptedException {
		// TODO Auto-generated method stub
		Set<WorkingMemoryAddress> newContainedPlaces = getContainedPlaceSet(room);
		Set<WorkingMemoryAddress> oldContainedPlaces = getStoredPlaceSet(roomAddr);

		for (WorkingMemoryAddress newPlace : newContainedPlaces) {
			// if it was already in the old set, we have nothing to do
			if (oldContainedPlaces.contains(newPlace))
				continue;

			try {
				lockEntry(newPlace, WorkingMemoryPermissions.LOCKEDOD);

				CASTIndependentFormulaDistributionsBelief<GroundedBelief> placeBelief = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class, getMemoryEntry(newPlace,
								GroundedBelief.class));
				FormulaDistribution roomProperty = FormulaDistribution.create();
				WorkingMemoryAddress roomBelief = getReferredBelief(new ComaRoomMatchingFunction(
						room.roomId));
				roomProperty
						.getDistribution()
						.add(
								WMPointer
										.create(
												roomBelief,
												CASTUtils
														.typeName(GroundedBelief.class))
										.get(), 1.0);
				placeBelief.getContent().put(ROOM_PROPERTY, roomProperty);
				overwriteWorkingMemory(newPlace, placeBelief.get());
			} finally {
				unlockEntry(newPlace);
			}

		}

	}

	private Set<WorkingMemoryAddress> getContainedPlaceSet(ComaRoom room)
			throws InterruptedException {
		Set<WorkingMemoryAddress> newContainedPlaces = new HashSet<WorkingMemoryAddress>();
		for (long place : room.containedPlaceIds) {
			log("trying to find the place belief for " + place
					+ " that is part of room " + room.roomId);
			WorkingMemoryAddress placeWMA = getReferredBelief(new PlaceMatchingFunction(
					place));
			newContainedPlaces.add(placeWMA);
		}
		return newContainedPlaces;
	}

	private Set<WorkingMemoryAddress> getStoredPlaceSet(
			WorkingMemoryAddress room) {
		Set<WorkingMemoryAddress> oldContainedPlaces = mapRoom2Places.get(room);
		if (oldContainedPlaces == null)
			oldContainedPlaces = new HashSet<WorkingMemoryAddress>();
		return oldContainedPlaces;
	}

	private void removeInvalidPlaceProperties(ComaRoom room,
			WorkingMemoryAddress roomAddr) throws InterruptedException,
			CASTException {
		Set<WorkingMemoryAddress> newContainedPlaces = getContainedPlaceSet(room);
		Set<WorkingMemoryAddress> oldContainedPlaces = getStoredPlaceSet(roomAddr);
		for (WorkingMemoryAddress p : oldContainedPlaces) {
			if (!newContainedPlaces.contains(p)) {
				removePlaceProperty(p);
			}
		}

	}

	private void removePlaceProperties(WorkingMemoryAddress roomAddr)
			throws CASTException {
		Set<WorkingMemoryAddress> oldContainedPlaces = getStoredPlaceSet(roomAddr);
		for (WorkingMemoryAddress place : oldContainedPlaces) {
			removePlaceProperty(place);
		}
	}

	private void removePlaceProperty(WorkingMemoryAddress place)
			throws CASTException {
		log("remove " + ROOM_PROPERTY + " from " + CASTUtils.toString(place));
		try {
			lockEntry(place, WorkingMemoryPermissions.LOCKEDOD);

			CASTIndependentFormulaDistributionsBelief<GroundedBelief> placeBelief = CASTIndependentFormulaDistributionsBelief
					.create(GroundedBelief.class, getMemoryEntry(place,
							GroundedBelief.class));
			placeBelief.getContent().remove(ROOM_PROPERTY);
			overwriteWorkingMemory(place, placeBelief.get());
		} finally {
			unlockEntry(place);
		}

	}

}
