/**
 * 
 */
package vision.components.mediators;

import java.util.HashMap;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Vector;

import Ice.Current;
import SpatialData.Place;
import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.IntegerValue;
import SpatialProperties.PlaceContainmentAgentProperty;
import SpatialProperties.PlaceContainmentObjectProperty;
import SpatialProperties.ProbabilityDistribution;
import SpatialProperties.PropertyValue;
import SpatialProperties.ValueProbabilityPair;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMView;
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
	private boolean propagateDelete;
	protected WMView<PlaceContainmentAgentProperty> agentPlace;

	/**
	 * 
	 */
	protected AbstractDetectedObjectMediator(Class<? extends T> type,
			boolean propagateDelete) {
		super();
		this.type = type;
		this.propagateDelete = propagateDelete;
		agentPlace = WMView.create(this, PlaceContainmentAgentProperty.class);
		queue = new WMEventQueue();
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
				log("working on event " + CASTUtils.toString(event));
				switch (event.operation) {
				case ADD: {
					T object = getMemoryEntry(event.address, type);
					long currentPlaceId = getPlace();
					log("current place is " + currentPlaceId);
					PlaceContainmentObjectProperty pcop = fillPlaceContainmentObjectProperty(
							new PlaceContainmentObjectProperty(),
							currentPlaceId);
					if (transform(pcop, object, event)) {
						String id = newDataID();
						propagationMap.put(event.address,
								new WorkingMemoryAddress(id,
										getSubarchitectureID()));
						addToWorkingMemory(id, pcop);
					}
					break;
				}
				case OVERWRITE: {
					PlaceContainmentObjectProperty pcop;

					WorkingMemoryAddress toBeUpdated = propagationMap
							.get(event.address);
					if (toBeUpdated != null) {
						pcop = getMemoryEntry(toBeUpdated,
								PlaceContainmentObjectProperty.class);
					} else {
						pcop = new PlaceContainmentObjectProperty();
					}
					T object = getMemoryEntry(event.address, type);
					long currentPlaceId = getPlace();
					log("current place is " + currentPlaceId);
					pcop = fillPlaceContainmentObjectProperty(
							new PlaceContainmentObjectProperty(),
							currentPlaceId);
					if (transform(pcop, object, event)) {
						overwriteWorkingMemory(toBeUpdated, pcop);
					}

					break;
				}

				case DELETE: {
					if (propagateDelete) {
						WorkingMemoryAddress toBeDeleted = propagationMap
								.get(event.address);
						if (toBeDeleted != null) {
							deleteFromWorkingMemory(toBeDeleted);
							propagationMap.remove(event.address);
						}
					}
					break;
				}
				}
			} catch (InterruptedException e) {
				logException(e);
			} catch (CASTException e) {
				logException(e);
			}

		}
	}

	private long getPlace() {
		try {
			PlaceContainmentAgentProperty pcap = agentPlace.values().iterator()
					.next();
			return ((IntegerValue) pcap.mapValue).value;
		} catch (NoSuchElementException e) {
			logException(e);
			return -1;
		}

	}

	private PlaceContainmentObjectProperty fillPlaceContainmentObjectProperty(
			PlaceContainmentObjectProperty placeContainmentObjectProperty,
			long currentPlaceId) {
		Vector<ValueProbabilityPair> data = new Vector<ValueProbabilityPair>(1);
		placeContainmentObjectProperty.mapValue = new IntegerValue(
				currentPlaceId);
		data.add(new ValueProbabilityPair(
				placeContainmentObjectProperty.mapValue, 1.0));
		placeContainmentObjectProperty.distribution = new DiscreteProbabilityDistribution(
				data);
		placeContainmentObjectProperty.label = "";
		placeContainmentObjectProperty.mapValueReliable = true;
		placeContainmentObjectProperty.objectId = -1;
		return placeContainmentObjectProperty;
	}

	protected abstract boolean transform(PlaceContainmentObjectProperty pcop,
			T object, WorkingMemoryChange event);

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		try {
			agentPlace.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(type,
				WorkingMemoryOperation.ADD), queue);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(type,
				WorkingMemoryOperation.OVERWRITE), queue);
		if (propagateDelete) {
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(type,
					WorkingMemoryOperation.DELETE), queue);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		try {
			removeChangeFilter(queue);
		} catch (SubarchitectureComponentException e) {
			logException(e);
		}
		super.stop();
	}

}
