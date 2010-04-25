/**
 * 
 */
package binder.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import comadata.ComaRoom;

import VisionData.VisualObject;
import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.builders.FeatureValueBuilder;
import binder.arch.BindingWorkingMemory;
import binder.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.helpers.ComaRoomMatchingFunction;
import binder.perceptmediator.transferfunctions.helpers.PlaceMatchingFunction;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMView;

/**
 * @author marc
 * 
 */
public class ComaRoomTransferFunction extends
		DependentDiscreteTransferFunction<ComaRoom> {

	public ComaRoomTransferFunction(ManagedComponent component,
			WMView<PerceptBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(ComaRoomTransferFunction.class));
		// TODO Auto-generated constructor stub
	}

	@Override
	protected Map<String, FeatureValue> getFeatureValueMapping(
			WorkingMemoryChange wmc, ComaRoom from) throws BeliefException,
			InterruptedException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		result.put("RoomId", FeatureValueBuilder
				.createNewStringValue(wmc.address.id));
		result.put("concepts", FeatureValueBuilder
				.createNewStringValue(from.concepts[0]));

		return result;
	}

	@Override
	public boolean transform(WorkingMemoryChange wmc, ComaRoom from,
			PerceptBelief perceptBelief) {
		boolean transformed = super.transform(wmc, from, perceptBelief);
		if (transformed) {
			try {
				for (long placeId : from.containedPlaceIds) {
					WorkingMemoryAddress wmaPlace = getReferredBelief(new PlaceMatchingFunction(
							placeId));
					PerceptBelief placeBelief = allBeliefs.get(wmaPlace);
					this.putDiscreteFeature(
							(CondIndependentDistribs) placeBelief.content,
							"in-room", new PointerValue(
									new WorkingMemoryAddress(perceptBelief.id,
											BindingWorkingMemory.BINDER_SA
											)));
					component.overwriteWorkingMemory(wmaPlace, placeBelief);
				}
			} catch (InterruptedException e) {
				component.logException(e);
				return false;
			} catch (CASTException e) {
				component.logException(e);
				return false;
			}

		}
		return transformed;
	}

}
