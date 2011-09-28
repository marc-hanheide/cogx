package facades;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import FrontierInterface.PlaceInterface;
import FrontierInterface.PlaceInterfacePrx;
import NavData.RobotPose2d;
import SpatialData.PathQueryStatus;
import SpatialData.PathTransitionCostRequest;
import SpatialData.Place;
import SpatialProperties.PlaceContainmentAgentProperty;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMEntryQueue;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMView;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;

/**
 * @author cogx
 * 
 */
public class SpatialFacade extends CASTHelper {

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
				// rooms.start();
			} catch (UnknownSubarchitectureException e1) {
				component.logException(e1);
			}
			WMEventQueue agentChangeEventQueue = new WMEventQueue();
			component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
					PlaceContainmentAgentProperty.class,
					WorkingMemoryOperation.ADD), agentChangeEventQueue);
			component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
					PlaceContainmentAgentProperty.class,
					WorkingMemoryOperation.OVERWRITE), agentChangeEventQueue);

			// initialize current place
			while (currentPlace == null && !interrupted())
				currentPlace = getPlaceInterface().getCurrentPlace();
			setPlace(currentPlace);

			while (!interrupted()) {
				try {
					// wait for the next change (wait 500ms to allow this to be
					// properly shutdown on interrupts)
					WorkingMemoryChange event = agentChangeEventQueue.poll(500,
							TimeUnit.MILLISECONDS);
					// if it was just a time out continue waiting
					if (event == null)
						continue;

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
					e.printStackTrace();
					break;
				}
			}
		}

	}

	private static final String PLACE_MANAGER = "place.manager";
	private static final String SPATIAL_SA = "spatial.sa";

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

	String placeManager = PLACE_MANAGER;

	String spatialSA = SPATIAL_SA;

	SpatialCheckThread spatialCheckThread;

	static private SpatialFacade singleton;

	Place currentPlace;

	Set<PlaceChangedHandler> placeCheckerCallables = null;
	PlaceInterfacePrx placeInterface;
	ManagedComponent component;
	WMView<Place> places;

	/**
	 * @param component
	 * @throws CASTException
	 */
	SpatialFacade(ManagedComponent component) throws CASTException {
		super(component);
		this.component = component;

		places = WMView.create(component, Place.class, SPATIAL_SA);

		this.spatialCheckThread = new SpatialCheckThread();

		placeInterface = null; // lazy init
		placeCheckerCallables = Collections
				.synchronizedSet(new HashSet<PlaceChangedHandler>());

	}

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

	
	public synchronized PlaceInterfacePrx getPlaceInterface() {

		while (placeInterface == null) {
			try {
				println("trying to connect to PlaceInterface server");

				placeInterface = component.getIceServer(PLACE_MANAGER,
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
	 * @return the placeManager
	 */
	public String getPlaceManager() {
		return placeManager;
	}

	/**
	 * get the current metric robot pose. NOTE: This is not the "official" way
	 * as the exact pose should be hidden to other components.
	 * 
	 * @return the pose
	 * @throws CASTException
	 */
	public RobotPose2d getPose() throws CASTException {
		List<RobotPose2d> poses = new ArrayList<RobotPose2d>(1);
		try {
			component.getMemoryEntries(RobotPose2d.class, poses, SPATIAL_SA);
			if (poses.size() < 1) {
				component.getLogger().warn(
						SpatialFacade.class.getSimpleName()
								+ " failed to get pose from working memory");
				throw new CASTException("there is no "
						+ RobotPose2d.class.getSimpleName() + " on "
						+ SPATIAL_SA);
			}
			return poses.get(0);
		} catch (CASTException e) {
			component.logException("in " + SpatialFacade.class.getSimpleName()
					+ " when trying to receive pose", e);
			throw (e);
		}
	}

	/**
	 * @return the spatialSA
	 */
	public String getSpatialSA() {
		return spatialSA;
	}

	/**
	 * query costs by planning a route to a place
	 * 
	 * @param from
	 *            place ID to start from
	 * @param to
	 *            place ID to go to
	 * @return costs
	 */
	public synchronized double queryCosts(long from, long to) {
		PathTransitionCostRequest ptcr = new PathTransitionCostRequest(from,
				to, 1.0, PathQueryStatus.QUERYPENDING);
		String id = component.newDataID();
		WMEntryQueue<PathTransitionCostRequest> resultQueue = new WMEntryQueue<PathTransitionCostRequest>(
				component, PathTransitionCostRequest.class);
		try {
			component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
					id, SPATIAL_SA, WorkingMemoryOperation.OVERWRITE),
					resultQueue);
			component.addToWorkingMemory(id, SPATIAL_SA, ptcr);
			while (ptcr.status == PathQueryStatus.QUERYPENDING) {
				WMEntryQueueElement<PathTransitionCostRequest> elem = resultQueue
						.take();
				log("PathTransitionCostRequest.status=" + ptcr.status.name());
				if (elem.getEntry() == null)
					return Double.MAX_VALUE;
				ptcr = elem.getEntry();
			}
			if (ptcr.status == PathQueryStatus.QUERYCOMPLETED) {
				log("we found a path and computed costs of " + ptcr.cost);
				if (ptcr.cost > 1E99)
					return Double.MAX_VALUE;
				return ptcr.cost;
			} else {
				println("could not find path from " + from + " to " + to);
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
				component.deleteFromWorkingMemory(id, SPATIAL_SA);
			} catch (CASTException e) {
				// ignore as it was only during GC
			}
		}

		return Double.MAX_VALUE;
	}

	/**
	 * register a callback that is called whenever the robot changes place
	 * 
	 * @param callable
	 */
	public synchronized void registerPlaceChangedCallback(
			PlaceChangedHandler callable) {
		placeCheckerCallables.add(callable);
	}

	private synchronized void setPlace(Place p) {
		currentPlace = p;
		notifyAll();
	}

	/**
	 * @param placeManager
	 *            the placeManager to set
	 */
	public void setPlaceManager(String placeManager) {
		this.placeManager = placeManager;
	}

	/**
	 * @param spatialSA
	 *            the spatialSA to set
	 */
	public void setSpatialSA(String spatialSA) {
		this.spatialSA = spatialSA;
	}

}
