package binder.components.perceptmediator.transferfunctions;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import org.apache.log4j.Logger;

import SpatialData.Place;
import SpatialProperties.ConnectivityPathProperty;
import binder.arch.BinderException;
import binder.autogen.beliefs.PerceptBelief;
import binder.autogen.distribs.CondIndependentDistribs;
import binder.autogen.distribs.ProbDistribution;
import binder.autogen.featurecontent.Feature;
import binder.autogen.featurecontent.FeatureValue;
import binder.builders.FeatureValueBuilder;
import binder.components.perceptmediator.PerceptBeliefManager;
import cast.CASTException;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.interfaces.ManagedComponent;
import castutils.castextensions.WMView;
import castutils.castextensions.WMView.ChangeHandler;

public class ConnectivityTransferFunction extends
		SimpleDiscreteTransferFunction<ConnectivityPathProperty, PerceptBelief> {

	WMView<Place> places;
	PerceptBeliefManager perceptBeliefManager;

	@Override
	Map<Feature, FeatureValue> getFeatureValueMapping(
			ConnectivityPathProperty from) throws InterruptedException {
		assert (from != null);
		Map<Feature, FeatureValue> result = new HashMap<Feature, FeatureValue>();
		// TODO: the features are stupid here!
		WorkingMemoryAddress wmaPlace1 = getReferredBelief(from.place1Id);
		WorkingMemoryAddress wmaPlace2 = getReferredBelief(from.place2Id);

		result.put(Feature.ConnectedTo1, FeatureValueBuilder
				.createNewStringValue(wmaPlace1.id));
		result.put(Feature.ConnectedTo2, FeatureValueBuilder
				.createNewStringValue(wmaPlace2.id));
		return result;
	}

	/**
	 * @param places
	 */
	public ConnectivityTransferFunction(WMView<Place> places, PerceptBeliefManager perceptBeliefManager) {
		super();
		this.places = places;
		this.perceptBeliefManager = perceptBeliefManager;

		places.registerHandler(new ChangeHandler<Place>() {

			@Override
			public void entryChanged(Map<WorkingMemoryAddress, Place> map,
					WorkingMemoryChange wmc, Place newEntry, Place oldEntry)
					throws CASTException {
				notifyWaitingThreads();

			}

		});

	}

	private synchronized void notifyWaitingThreads() {
		notifyAll();
	}

	private synchronized WorkingMemoryAddress getReferredBelief(long place1Id)
			throws InterruptedException {

		while (true) {
			for (Entry<WorkingMemoryAddress, Place> entry : places.entrySet()) {
				if (entry.getValue().id == place1Id) {
					
					try {
						WorkingMemoryAddress beliefWMA = perceptBeliefManager.read().percept2Belief.get(entry.getKey());
						if (beliefWMA!=null)
							return beliefWMA;
						else
							throw (new IllegalStateException("found belief, but can't find it in map... this MUST NOT happen!"));
					} catch (CASTException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					return entry.getKey();
				}
			}
			wait();
		}
	}

	@Override
	public PerceptBelief createBelief(String id, CASTTime curTime)
			throws BinderException {
		PerceptBelief basePb = super.createBelief(id, curTime);
		PerceptBelief newPb = new PerceptBelief();
		newPb.estatus = basePb.estatus;
		newPb.frame = basePb.frame;
		newPb.hist = basePb.hist;
		newPb.id = id;
		newPb.content = null;

		return newPb;
	}

}
