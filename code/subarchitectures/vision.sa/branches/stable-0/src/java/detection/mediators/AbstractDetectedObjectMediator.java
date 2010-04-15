/**
 * 
 */
package detection.mediators;

import java.util.HashMap;
import java.util.Map;

import SpatialData.Place;
import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.ProbabilityDistribution;
import SpatialProperties.PropertyValue;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;
import castutils.facades.SpatialFacade;

/**
 * @author marc
 * 
 */
public abstract class AbstractDetectedObjectMediator<T extends Ice.ObjectImpl>
		extends ManagedComponent {

	protected Class<? extends T> type;
	private WMEventQueue queue;
	private Map<WorkingMemoryAddress, WorkingMemoryAddress> propagationMap;

	/**
	 * 
	 */
	protected AbstractDetectedObjectMediator(Class<? extends T> type) {
		super();
		this.type=type;
		queue=new WMEventQueue();
		propagationMap = new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		while (isRunning()) {
			try {
				WorkingMemoryChange event = queue.take();
				switch (event.operation) {
				case ADD: {
					T object = getMemoryEntry(event.address, type);

					Place currentPlace = SpatialFacade.get(this).getPlace();
					PlaceContainmentObjectProperty pcop = new PlaceContainmentObjectProperty(
							-1, new ProbabilityDistribution(),
							new PropertyValue(), true);
					if (transform(pcop, object)) {
						String id = newDataID();
						propagationMap.put(event.address,
								new WorkingMemoryAddress(id,
										getSubarchitectureID()));
						addToWorkingMemory(newDataID(), pcop);
					}
					break;
				}
				case OVERWRITE: {
					WorkingMemoryAddress toBeUpdated = propagationMap
							.get(event.address);
					if (toBeUpdated != null) {
						PlaceContainmentObjectProperty pcop = getMemoryEntry(
								toBeUpdated,
								PlaceContainmentObjectProperty.class);
						T object = getMemoryEntry(event.address, type);
						Place currentPlace = SpatialFacade.get(this).getPlace();
						if (transform(pcop, object)) {
							overwriteWorkingMemory(toBeUpdated, pcop);
						}
					}
					break;
				}

				case DELETE: {
					WorkingMemoryAddress toBeDeleted = propagationMap
							.get(event.address);
					if (toBeDeleted != null) {
						deleteFromWorkingMemory(toBeDeleted);
						propagationMap.remove(event.address);
					}
					break;
				}
				}
			} catch (InterruptedException e) {
				getLogger().error("interrupted ", e);
			} catch (CASTException e) {
				getLogger().error("CASTexception ", e);
			}

		}
	}

	protected abstract boolean transform(PlaceContainmentObjectProperty pcop,
			T object);

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(type,
				WorkingMemoryOperation.OVERWRITE), queue);
	}

}
