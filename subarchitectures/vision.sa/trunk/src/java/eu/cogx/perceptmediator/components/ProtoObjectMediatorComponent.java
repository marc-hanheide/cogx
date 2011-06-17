package eu.cogx.perceptmediator.components;

import VisionData.ProtoObject;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ProtoObjectTransferFunction;

public class ProtoObjectMediatorComponent extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<ProtoObject, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this, ProtoObject.class,
				PerceptBelief.class, new ProtoObjectTransferFunction(this));
	}

}
