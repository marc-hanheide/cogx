package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;

import SpatialData.Place;
import SpatialProperties.ConnectivityPathProperty;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueDistribution;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.builders.FeatureValueBuilder;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;

public class ConnectivityTransferFunction extends
		DependentDiscreteTransferFunction<ConnectivityPathProperty, PerceptBelief> {

	class PlaceMatchingFunction implements ReferenceMatchingFunction<PerceptBelief> {
		
		private long placeId;
		
		
		/**
		 * @param placeId
		 */
		public PlaceMatchingFunction(long placeId) {
			super();
			this.placeId = placeId;
		}


		@Override
		public boolean matches(PerceptBelief r) {
			if (r.type.equals(Place.class.getCanonicalName())) {
				assert (r.content instanceof CondIndependentDistribs);
				CondIndependentDistribs dist = (CondIndependentDistribs) r.content;
				FeatureValueDistribution fv = (FeatureValueDistribution) dist.distribs.get("PlaceId");
				IntegerValue idVal = (IntegerValue) fv.values.get(0).val;
				
				return idVal.val == placeId;
				
			}
			else {
				return false;
			}
		}
		
	}
	
	@Override
	Map<String, FeatureValue> getFeatureValueMapping(
			final ConnectivityPathProperty from) throws InterruptedException {
		assert (from != null);
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();

	
		WorkingMemoryAddress wmaPlace1 = getReferredBelief(new PlaceMatchingFunction(from.place1Id));
		WorkingMemoryAddress wmaPlace2 = getReferredBelief(new PlaceMatchingFunction(from.place2Id));

		result.put("ConnectedTo1", FeatureValueBuilder
				.createNewStringValue(wmaPlace1.id));
		result.put("ConnectedTo2", FeatureValueBuilder
				.createNewStringValue(wmaPlace2.id));
		return result;
	}

	/**
	 * @param perceptBeliefs
	 */
	public ConnectivityTransferFunction(WMView<PerceptBelief> perceptBeliefs) {
		super(perceptBeliefs);

	}


}
