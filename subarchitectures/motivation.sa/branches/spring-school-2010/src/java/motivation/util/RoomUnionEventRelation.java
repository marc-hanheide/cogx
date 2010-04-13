/**
 * 
 */
package motivation.util;

import java.util.List;

import SpatialData.Place;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.StringValue;
import binder.components.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import castutils.castextensions.CausalEventMonitor;
import castutils.facades.BinderFacade;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

import comadata.ComaRoom;

/**
 * This class implements an EventRelation between ComaRooms and
 * UnionConfiguration updates. It allows to encapsulate the propagation of place
 * information to the binder. Usually it is simply used as a Monitor.
 * Instantiate one, then start() it and run waitForPropagation() whenever you
 * have to make sure that all Place changes have been propagated to the binder.
 * 
 * @author marc
 * 
 * @sa compare()
 * 
 */
public class RoomUnionEventRelation extends
		CausalEventMonitor<ComaRoom, Belief> {

	/**
	 * generate a ComaRoomUnionEventRelation that uses the given component. It
	 * adds also the required filters to the super class
	 * {@link CausalEventMonitor}.
	 * 
	 * 
	 * @param component
	 *            the component object to use for memory access
	 */
	public RoomUnionEventRelation(ManagedComponent component) {
		super(component);
		addTriggerFilter(ChangeFilterFactory
				.createGlobalTypeFilter(ComaRoom.class));
		addImplicationFilter(ChangeFilterFactory
				.createGlobalTypeFilter(Belief.class));

	}

	/**
	 * compares the two memory changes and return true if they are equal.
	 * "Equal" for a {@link ComaRoom} and {@link UnionConfiguration} means that
	 * the place_id in a union is referring to the place id. It implements the
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

		// if we have a delete event... simply return true
		if (wmcTrigger.operation == WorkingMemoryOperation.DELETE) {
			return true;
		}

		try {
			ComaRoom room = component.getMemoryEntry(wmcTrigger.address,
					ComaRoom.class);
			Belief belief = component.getMemoryEntry(wmcImplication.address,
					Belief.class);
			if (wmcTrigger.operation == WorkingMemoryOperation.ADD
					|| wmcTrigger.operation == WorkingMemoryOperation.OVERWRITE) {
				// on add or overwrite we expect to find the place in the
				// unions
				return findComaRoomIdInUnions(belief, room.roomId);
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
	 * @param searchedRoomID
	 *            the place_id we are looking for
	 * @return /** first adds specific Filters for the triggers and implications
	 *         and then starts the monitoring
	 * 
	 * @see motivation.util.castextensions.CausalEventMonitor#start()
	 */
	private boolean findComaRoomIdInUnions(Belief belief, long searchedRoomID) {
		if (belief.type.equals(CASTUtils.typeName(ComaRoom.class))) {
			List<FeatureValue> fvl = BinderFacade
					.get(component)
					.getFeatureValue(belief, PlaceMatchingFunction.PLACE_ID);
			int placeId = Integer.parseInt(((StringValue) fvl.get(0)).val);
			if (placeId == searchedRoomID) {
				component.debug("found corresponding ID");
				return true;
			}

		}
		return false;
	}
}
