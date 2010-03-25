/**
 * 
 */
package motivation.util;

import comadata.ComaRoom;

import castutils.castextensions.CausalEventMonitor;
import SpatialData.Place;
import binder.autogen.core.Feature;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.StringValue;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * This class implements an EventRelation between Places and UnionConfiguration
 * updates. It allows to encapsulate the propagation of place information to the
 * binder. Usually it is simply used as a Monitor. Instantiate one, then start()
 * it and run waitForPropagation() whenever you have to make sure that all Place
 * changes have been propagated to the binder.
 * 
 * @author marc
 * 
 * @sa compare()
 * 
 */
public class PlaceUnionEventRelation extends
		CausalEventMonitor<Place, UnionConfiguration> {

	/**
	 * generate a PlaceUnionEventRelation that uses the given component. It adds
	 * also the required filters to the super class {@link CausalEventMonitor}.
	 * 
	 * 
	 * @param component
	 *            the component object to use for memory access
	 */
	public PlaceUnionEventRelation(ManagedComponent component) {
		super(component);
		addTriggerFilter(ChangeFilterFactory
				.createGlobalTypeFilter(Place.class));
		addImplicationFilter(ChangeFilterFactory
				.createGlobalTypeFilter(UnionConfiguration.class));

	}

	/**
	 * compares the two memory changes and return true if they are equal.
	 * "Equal" for a {@link Place} and {@link UnionConfiguration} means that the
	 * place_id in a union is referring to the place id. It implements the
	 * abstract {@link CausalEventMonitor}.compare() method. The check performed
	 * here returns true iff either the place has been deleted (this is based on
	 * the assumption that this doesn't need to be propagated to the binder
	 * because it's about the removal of information) or a {@link Union} with a
	 * place_id matching the one of the changed {@link Place} has been found.
	 */
	@Override
	protected boolean compare(WorkingMemoryChange wmcTrigger,
			WorkingMemoryChange wmcImplication) throws CASTException {
		component.debug("comparing changes");

		UnionConfiguration unionConfiguration = component.getMemoryEntry(
				wmcImplication.address, UnionConfiguration.class);

		// sanity check...
		if (unionConfiguration.includedUnions == null)
			return false;
		// if we have a delete event... simply return true
		if (wmcTrigger.operation == WorkingMemoryOperation.DELETE) {
			return true;
		}

		try {
			Place place = component.getMemoryEntry(wmcTrigger.address,
					Place.class);
			if (wmcTrigger.operation == WorkingMemoryOperation.ADD
					|| wmcTrigger.operation == WorkingMemoryOperation.OVERWRITE) {
				// on add or overwrite we expect to find the place in the
				// unions

				return findPlaceIdInUnions(unionConfiguration.includedUnions,
						place.id);
			}
		} catch (DoesNotExistOnWMException e) {
			component
					.log("the trigger does not exist anymore, so consider it being propagated");
			return true;
		}

		return false;
	}

	/**
	 * this performs the search for a specific place_id on all existing unions
	 * 
	 * @param unions
	 *            unions to be searched
	 * @param searchedPlaceID
	 *            the place_id we are looking for
	 * @return /** first adds specific Filters for the triggers and implications
	 *         and then starts the monitoring
	 * 
	 * @see motivation.util.castextensions.CausalEventMonitor#start()
	 */
	private boolean findPlaceIdInUnions(Union[] unions, long searchedPlaceID) {
		for (Union union : unions) {
			for (Feature f : union.features) {
				if (f.featlabel.equals("place_id")) {
					int placeId = Integer
							.parseInt(((StringValue) f.alternativeValues[0]).val);
					if (placeId == searchedPlaceID) {
						component.debug("found corresponding ID");
						return true;
					}
				}
			}
		}
		return false;
	}

}
