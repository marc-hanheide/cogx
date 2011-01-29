package facades;

import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

//import comadata.ComaRoom;

import FrontierInterface.PlaceInterface;
import FrontierInterface.PlaceInterfacePrx;
import Ice.ObjectImpl;
import SpatialData.PathQueryStatus;
import SpatialData.PathTransitionCostRequest;
import SpatialData.Place;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMEntryQueue;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMView;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import castutils.castextensions.WMEntrySet.ChangeHandler;

public class SpatialFacade extends CASTHelper implements ChangeHandler {

	public interface PlaceChangedHandler {
		void update(Place p);
	}

	class SpatialCheckThread extends Thread {
		/**
		 * @param arg0
		 */
		SpatialCheckThread() {
			super(SpatialFacade.class.getSimpleName() + "singletonThread");
		}

		/**
		 * run method of the tread check for new places every second
		 */
		@Override
		public void run() {
			try {
				places.start();
//				rooms.start();
			} catch (UnknownSubarchitectureException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			WMEventQueue agentChangeEventQueue = new WMEventQueue();
			// TODO reintroduce handling of place changing
//			component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
//					PlaceContainmentAgentProperty, WorkingMemoryOperation.ADD), agentChangeEventQueue);
//			component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
//					PlaceContainmentAgentProperty.class, WorkingMemoryOperation.OVERWRITE), agentChangeEventQueue);

			
			// initialize current place
			while (currentPlace == null && !interrupted())
				currentPlace = getPlaceInterface().getCurrentPlace();
			setPlace(currentPlace);

			while (!interrupted()) {
				try {
					// wait for the next change
					agentChangeEventQueue.take();

					Place place = getPlaceInterface().getCurrentPlace();

					if (place == null || place.id != currentPlace.id) {
						setPlace(place);
						for (PlaceChangedHandler c : placeCheckerCallables) {
							c.update(place);
						}
					}
				} catch (InterruptedException e) {
					break;
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
					break;
				}
			}
		}

	}
	
	SpatialCheckThread spatialCheckThread;
	static private SpatialFacade singleton;
	private Place currentPlace;
	private Set<PlaceChangedHandler> placeCheckerCallables = null;
	private PlaceInterfacePrx placeInterface;
	ManagedComponent component;
	WMView<Place> places;

	//	WMView<ComaRoom> rooms;

	// TODO: reintroduce rooms!
	//	/**
//	 * returns room a given place is in
//	 * 
//	 * @return room the place is in
//	 */
//	public ComaRoom getRoom(Place p) {
//		for (ComaRoom cr : new HashSet<ComaRoom>(rooms.values())) {
//			for (int i = 0; i < cr.containedPlaceIds.length; i++) {
//				if (cr.containedPlaceIds[i] == p.id) {
//					return cr;
//				}
//
//			}
//		}
//		return null;
//	}

//	/**
//	 * returns all places belonging to the room
//	 * 
//	 * @return set of places
//	 */
//	public Set<Place> getPlaces(ComaRoom cr) {
//		Set<Place> result = new HashSet<Place>();
//		for (int i = 0; i < cr.containedPlaceIds.length; i++) {
//			for (Place p : new HashSet<Place>(places.values())) {
//				if (p.id == cr.containedPlaceIds[i]) {
//					result.add(p);
//				}
//			}
//		}
//		return result;
//	}

//	/**
//	 * returns current room
//	 * 
//	 * @return current room
//	 */
//	public ComaRoom getRoom() throws InterruptedException {
//		Place currentPlace = getPlace();
//		return getRoom(currentPlace);
//	}

	/**
	 * retrieves the current place
	 * 
	 * @return the place
	 * @throws InterruptedException
	 */
	public synchronized Place getPlace() throws InterruptedException {
		if (!spatialCheckThread.isAlive())
			spatialCheckThread.start();

		if (currentPlace == null)
			wait();
		return currentPlace;
	}

	private synchronized void setPlace(Place p) {
		currentPlace = p;
		notifyAll();
	}

	/** get a proxy to the placeInterface as a singleton */
	synchronized PlaceInterfacePrx getPlaceInterface() {

		while (placeInterface == null) {
			try {
				println("trying to connect to PlaceInterface server");

				placeInterface = component.getIceServer("place.manager",
						PlaceInterface.class, PlaceInterfacePrx.class);
				log("PlaceInterface initialized... ");
				// Thread.sleep(3000);
				// component

				// .println("placeInterface.getCurrentPlace successfully called... connection alright");

				break;
			} catch (CASTException e) {
				component
						.println("could not yet initialize PlaceInterface... trying again in a second: "
								+ e.message);
				e.printStackTrace();
				placeInterface = null;
			} catch (Ice.UnknownException e) {
				component
						.println("could not yet initialize PlaceInterface... trying again in a second: "
								+ e.getMessage());
				e.printStackTrace();
				placeInterface = null;
			}
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e1) {
				e1.printStackTrace();
			}
		}
		return placeInterface;
	}


	/**
	 * get the singleton reference
	 * 
	 * @param component
	 * @return the singleton
	 * @throws CASTException
	 */
	public static SpatialFacade get(ManagedComponent component)
			throws CASTException {
		if (singleton == null) {
			singleton = new SpatialFacade(component);
		}
		return singleton;
	}

	public synchronized void registerPlaceChangedCallback(
			PlaceChangedHandler callable) {
		placeCheckerCallables.add(callable);
	}

	/**
	 * @param component
	 * @throws CASTException
	 */
	SpatialFacade(ManagedComponent component) throws CASTException {
		super(component);
		this.component = component;
		
		
		places = WMView.create(component, Place.class, "spatial.sa");
//		rooms = WMView.create(component, ComaRoom.class,"coma");
		
		
		this.spatialCheckThread = new SpatialCheckThread();

		placeInterface = null; // lazy init
		placeCheckerCallables = Collections
				.synchronizedSet(new HashSet<PlaceChangedHandler>());
		
	}

	public synchronized double queryCosts(long from, long to) {
		PathTransitionCostRequest ptcr = new PathTransitionCostRequest(from,
				to, 1.0, PathQueryStatus.QUERYPENDING);
		String id = component.newDataID();
		WMEntryQueue<PathTransitionCostRequest> resultQueue = new WMEntryQueue<PathTransitionCostRequest>(component, PathTransitionCostRequest.class);
		try {
			component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
					id, "spatial.sa", WorkingMemoryOperation.OVERWRITE),
					resultQueue);
			component.addToWorkingMemory(id, "spatial.sa", ptcr);
			while (ptcr.status == PathQueryStatus.QUERYPENDING) {
				WMEntryQueueElement<PathTransitionCostRequest> elem = resultQueue.take();
				log("PathTransitionCostRequest.status="
						+ ptcr.status.name());
				if (elem.getEntry() == null)
					return Double.MAX_VALUE;
				ptcr = elem.getEntry();
			}
			if (ptcr.status == PathQueryStatus.QUERYCOMPLETED) {
				log("we found a path and computed costs of "
						+ ptcr.cost);
				if (ptcr.cost > 1E99)
					return Double.MAX_VALUE;
				return ptcr.cost;
			} else {
				println("could not find path from " + from + " to "
						+ to);
			}

		} catch (CASTException e) {
			println("CASTException in SpatialFacade::queryCosts");
			e.printStackTrace();
		} catch (InterruptedException e) {
			component
					.println("InterruptedException in SpatialFacade::queryCosts");
			e.printStackTrace();
		} finally {
			try {
				component.deleteFromWorkingMemory(id, "spatial.sa");
			} catch (CASTException e) {
				// ignore as it was only during GC
			}
		}

		return Double.MAX_VALUE;
	}

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
			WorkingMemoryChange wmc, ObjectImpl newMotive, ObjectImpl oldMotive)
			throws CASTException {

	}

}
