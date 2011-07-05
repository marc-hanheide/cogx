package eu.cogx.perceptmediator.components;

import VisionData.ProtoObject;
import VisionData.ViewCone;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ProtoObjectTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ViewConeTransferFunction;

public class ViewConeMediatorComponent extends PerceptMediatorComponent {

	@Override
	protected PerceptBindingMediator<ViewCone, PerceptBelief> getMediator() {
		return PerceptBindingMediator.create(this, ViewCone.class,
				PerceptBelief.class, new ViewConeTransferFunction(this));
	}

}
