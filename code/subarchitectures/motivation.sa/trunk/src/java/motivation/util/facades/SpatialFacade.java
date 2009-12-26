package motivation.util.facades;

import java.util.Collections;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import comadata.ComaRoom;

import motivation.util.castextensions.WMEntryQueue;
import motivation.util.castextensions.WMEntrySet;
import motivation.util.castextensions.WMEntryQueue.WMEntryQueueElement;
import motivation.util.castextensions.WMEntrySet.ChangeHandler;
import FrontierInterface.PlaceInterface;
import FrontierInterface.PlaceInterfacePrx;
import Ice.ObjectImpl;
import SpatialData.PathQueryStatus;
import SpatialData.PathTransitionCostRequest;
import SpatialData.Place;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class SpatialFacade extends Thread implements ChangeHandler {

	public interface PlaceChangedHandler {
		void update(Place p);
	}

	static private SpatialFacade singleton;
	private Place currentPlace;
	private Set<PlaceChangedHandler> placeCheckerCallables = null;
	private PlaceInterfacePrx placeInterface;
	ManagedComponent component;
	WMEntrySet places;
	WMEntrySet rooms;

	/** returns room a given place is in 
	 * @return room the place is in
	 */
	public ComaRoom getRoom(Place p) {
		for (ObjectImpl oi : rooms.values()) {
			ComaRoom cr = (ComaRoom) oi;
			for (int i=0; i<cr.containedPlaceIds.length; i++) {
				if (cr.containedPlaceIds[i]==p.id) {
					return cr;
				}
				
			}
		}
		return null;
	}

	/** returns current room 
	 * @return current room
	 */
	public ComaRoom getRoom() throws InterruptedException {
		Place currentPlace = getPlace();
		return getRoom(currentPlace);
	}
	
	/** retrieves the current place
	 * 
	 * @return the place
	 * @throws InterruptedException
	 */
	public synchronized Place getPlace() throws InterruptedException {
		if (currentPlace==null)
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
				component.println("trying to connect to PlaceInterface server");

				placeInterface = component.getIceServer("place.manager",
						PlaceInterface.class, PlaceInterfacePrx.class);
				component.println("PlaceInterface initialized... ");
//				Thread.sleep(3000);
//				component
//						.println("placeInterface.getCurrentPlace successfully called... connection alright");

				break;
			} catch (CASTException e) {
				component
						.println("could not yet initialize PlaceInterface... trying again in a second: " +e.message);
				e.printStackTrace();
				placeInterface=null;
			} catch (Ice.UnknownException e) {
				component
						.println("could not yet initialize PlaceInterface... trying again in a second: "+e.getMessage());
				e.printStackTrace();
				placeInterface=null;
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
	 * run method of the tread check for new places every second
	 */
	@Override
	public void run() {
		places.start();
		rooms.start();
		
		// initialize current place
		while (currentPlace == null && !interrupted())
			currentPlace = getPlaceInterface().getCurrentPlace();
		setPlace(currentPlace);
		
		while (!interrupted()) {
			try {
				Place place = getPlaceInterface().getCurrentPlace();

				if (place == null || place.id != currentPlace.id) {
					setPlace(place);
					for (PlaceChangedHandler c : placeCheckerCallables) {
						c.update(place);
					}
				}
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				break;
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				break;
			}
		}
	}

	/** get the singleton reference 
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
		super(SpatialFacade.class.getSimpleName() + "singletonThread");
		this.component = component;
		places = WMEntrySet.create(component, Place.class);
		rooms = WMEntrySet.create(component, ComaRoom.class);
		
		placeInterface = null; // lazy init
		placeCheckerCallables = Collections
				.synchronizedSet(new HashSet<PlaceChangedHandler>());
	}

	public synchronized double queryCosts(long from, long to) {
		PathTransitionCostRequest ptcr = new PathTransitionCostRequest(from,
				to, 1.0, PathQueryStatus.QUERYPENDING);
		String id = component.newDataID();
		WMEntryQueue resultQueue = new WMEntryQueue(component);
		try {
			component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
					id, "spatial.sa", WorkingMemoryOperation.OVERWRITE),
					resultQueue);
			component.addToWorkingMemory(id, "spatial.sa", ptcr);
			while (ptcr.status == PathQueryStatus.QUERYPENDING) {
				WMEntryQueueElement elem = resultQueue.take();
				component.log("PathTransitionCostRequest.status="
						+ ptcr.status.name());
				if (elem.getEntry() == null)
					return Double.MAX_VALUE;
				ptcr = (PathTransitionCostRequest) elem.getEntry();
			}
			if (ptcr.status == PathQueryStatus.QUERYCOMPLETED) {
				component.log("we found a path and computed costs of "
						+ ptcr.cost);
				if (ptcr.cost>1E99)
					return Double.MAX_VALUE;
				return ptcr.cost;
			} else {
				component.println("could not find path from " + from + " to "
						+ to);
			}

		} catch (CASTException e) {
			component.println("CASTException in SpatialFacade::queryCosts");
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
