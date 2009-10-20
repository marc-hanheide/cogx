package motivation.util.facades;

import java.util.Map;

import motivation.util.castextensions.WMEntryQueue;
import motivation.util.castextensions.WMEntrySet;
import motivation.util.castextensions.WMEntryQueue.WMEntryQueueElement;
import motivation.util.castextensions.WMEntrySet.ChangeHandler;
import FrontierInterface.PlaceInterfacePrx;
import Ice.ObjectImpl;
import SpatialData.PathQueryStatus;
import SpatialData.PathTransitionCostRequest;
import SpatialData.Place;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class SpatialFacade implements ChangeHandler {
	ManagedComponent component;
	PlaceInterfacePrx placeInterface;

	WMEntrySet places;

	/**
	 * @param component
	 */
	public SpatialFacade(ManagedComponent component,
			PlaceInterfacePrx placeInterface) {
		super();
		this.component = component;
		this.placeInterface = placeInterface;
		places = WMEntrySet.create(component, Place.class);
	}

	public void start() {
		places.setHandler(this);
		places.start();
	}

	public Place getCurrentPlace() {

		return placeInterface.getCurrentPlace();
	}

	public double queryCosts(long from, long to) {
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

		return Long.MAX_VALUE;
	}

	@Override
	public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
			WorkingMemoryChange wmc, ObjectImpl newMotive, ObjectImpl oldMotive)
			throws CASTException {

	}

}
