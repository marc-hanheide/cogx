/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.perceptmediator.components.abstr;

import java.util.Map;
import java.util.Map.Entry;

import SpatialProperties.DiscreteProbabilityDistribution;
import SpatialProperties.FloatValue;
import SpatialProperties.PlaceProperty;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import castutils.castextensions.WMContentWaiter;
import castutils.castextensions.WMEventQueue;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class PlacePropertyMediator<From extends PlaceProperty> extends
		ManagedComponent {

	private final WMView<GroundedBelief> allPercepts;

	private WMEventQueue entryQueue;

	private final WMContentWaiter<GroundedBelief> waitingBeliefReader;
	/**
	 * the from type
	 */
	protected final Class<From> fromType;

	/**
	 * @param fromType
	 */
	public PlacePropertyMediator(Class<From> fromType) {
		super();
		this.fromType = fromType;
		this.allPercepts = WMView.create(this, GroundedBelief.class);
		this.waitingBeliefReader = new WMContentWaiter<GroundedBelief>(
				allPercepts);
		entryQueue = new WMEventQueue();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
	}

	protected abstract boolean fillValues(
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> create,
			From from);

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
					switch (ev.operation) {
					case ADD:
					case OVERWRITE: {
						From from = getMemoryEntry(ev.address, fromType);
						Entry<WorkingMemoryAddress, GroundedBelief> referredPlace = waitingBeliefReader
								.read(new PlaceMatchingFunction(from.placeId));
						assert (referredPlace != null);
						assert (referredPlace.getKey() != null);

						try {
							lockEntry(referredPlace.getKey(),
									WorkingMemoryPermissions.LOCKEDOD);
							GroundedBelief pb = getMemoryEntry(referredPlace
									.getKey(), GroundedBelief.class);
							debug("got referred place: "
									+ referredPlace.getKey().id);
							if (fillValues(
									CASTIndependentFormulaDistributionsBelief
											.create(GroundedBelief.class, pb),
									from)) {
								overwriteWorkingMemory(referredPlace.getKey(),
										pb);
							}
						} finally {
							unlockEntry(referredPlace.getKey());
						}
						break;
					}
					case DELETE: {
						break;
					}

					}
				} catch (CASTException e) {
					logException("in run: ", e);
				}
			}
		} catch (InterruptedException e) {
			getLogger().warn("interrupted in run: ", e);
		} finally {
			try {
				removeChangeFilter(entryQueue);
			} catch (SubarchitectureComponentException e) {
				getLogger().error("while removing change filter: ", e);
			}
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(fromType,
				WorkingMemoryOperation.ADD), entryQueue);
		addChangeFilter(ChangeFilterFactory.createTypeFilter(fromType,
				WorkingMemoryOperation.OVERWRITE), entryQueue);
		// addChangeFilter(ChangeFilterFactory.createTypeFilter(fromType,
		// WorkingMemoryOperation.DELETE), entryQueue);
		try {
			allPercepts.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	protected static double getFirstPropertyValue(
			DiscreteProbabilityDistribution _probabilityDistribution) {
		if (_probabilityDistribution == null)
			return 0.0;
		else
			return ((FloatValue) _probabilityDistribution.data[0].value).value;
	}

	protected static double getFirstPropertyValue(PlaceProperty _property) {
		if (_property == null)
			return 0.0;
		else
			return getFirstPropertyValue((DiscreteProbabilityDistribution) _property.distribution);
	}

}
