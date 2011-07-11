package eu.cogx.perceptmediator.components;

import VisionData.ProtoObject;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ProtoObjectTransferFunction;

public class ProtoObjectMediatorComponent extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<ProtoObject, GroundedBelief> getMediator() {
		return PerceptBindingMediator.create(this, ProtoObject.class,
				GroundedBelief.class, new ProtoObjectTransferFunction(this));
	}

}
