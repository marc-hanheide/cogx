package eu.cogx.perceptmediator.components;

import VisionData.ProtoObject;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ProtoObjectTransferFunction;

public class ProtoObjectMediatorComponent extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public ProtoObjectMediatorComponent() {
		super(GroundedBelief.class);

	}

	@Override
	protected PerceptBindingMediator<ProtoObject, GroundedBelief> getMediator(
			String _toSA) {
		return PerceptBindingMediator.create(this, _toSA, ProtoObject.class,
				GroundedBelief.class, new ProtoObjectTransferFunction(this, allBeliefs));
	}

}
