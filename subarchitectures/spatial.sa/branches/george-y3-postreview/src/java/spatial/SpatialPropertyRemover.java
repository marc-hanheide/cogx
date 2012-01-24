/**
 * 
 */
package spatial;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import SpatialData.Place;
import SpatialProperties.ConnectivityPathProperty;
import SpatialProperties.PathProperty;
import SpatialProperties.PlaceProperty;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

/**
 * monitors the places and whenever a place is deleted it removes all related
 * PlaceProperty and PathProperty objects from WM
 * 
 * @author hanheidm
 * 
 */
public class SpatialPropertyRemover extends ManagedComponent implements
		WorkingMemoryChangeReceiver {

	Map<WorkingMemoryAddress, Place> placeMap = Collections
			.synchronizedMap(new HashMap<WorkingMemoryAddress, Place>());

	@Override
	protected void start() {
		println(SpatialPropertyRemover.class.getSimpleName()+" starts listening for places");
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Place.class,
				WorkingMemoryOperation.DELETE), this);
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Place.class,
				WorkingMemoryOperation.ADD), this);
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange wmc)
			throws CASTException {
		try {
			switch (wmc.operation) {
			case ADD:
				Place addedPlace = getMemoryEntry(wmc.address, Place.class);
				placeMap.put(wmc.address, addedPlace);
				break;
			case DELETE:
				Place removedPlace = placeMap.remove(wmc.address);
				removeAllRelatedProperties(removedPlace);
				break;
			}

		} catch (CASTException e) {
			logException(e);
		}

	}

	private void removeAllRelatedProperties(Place removedPlace)
			throws CASTException {
		removePlaceProperties(removedPlace);
		removePathProperties(removedPlace);
	}

	private void removePlaceProperties(Place removedPlace)
			throws PermissionException {
		List<CASTData<PlaceProperty>> placeProps = new ArrayList<CASTData<PlaceProperty>>();
		getMemoryEntriesWithData(PlaceProperty.class, placeProps, 0);
		log("check all " + placeProps.size() + " "
				+ PlaceProperty.class.getSimpleName() + " referring to place "
				+ removedPlace.id);
		for (CASTData<PlaceProperty> prop : placeProps) {
			if (prop.getData().placeId == removedPlace.id) {
				log("have to remove PlaceProperty " + prop.getID()
						+ " because place " + removedPlace.id
						+ " has been removed");
				try {
					deleteFromWorkingMemory(prop.getID());
				} catch (DoesNotExistOnWMException e) {
					getLogger().warn("property has been removed earlier", e);
				}
			}
		}
	}

	private void removePathProperties(Place removedPlace)
			throws PermissionException {
		List<CASTData<ConnectivityPathProperty>> pathProps = new ArrayList<CASTData<ConnectivityPathProperty>>();
		getMemoryEntriesWithData(ConnectivityPathProperty.class, pathProps, 0);
		log("check all " + pathProps.size() + " "
				+ PathProperty.class.getSimpleName() + " referring to place "
				+ removedPlace.id);
		for (CASTData<ConnectivityPathProperty> prop : pathProps) {
			if (prop.getData().place1Id == removedPlace.id
					|| prop.getData().place2Id == removedPlace.id) {
				log("have to remove PathProperty " + prop.getID()
						+ " because place " + removedPlace.id
						+ " has been removed");
				try {
					deleteFromWorkingMemory(prop.getID());
				} catch (DoesNotExistOnWMException e) {
					getLogger().warn("property has been removed earlier", e);
				}
			}
		}
	}

}
