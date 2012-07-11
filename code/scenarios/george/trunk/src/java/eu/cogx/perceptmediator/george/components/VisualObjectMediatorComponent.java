package eu.cogx.perceptmediator.george.components;


import VisionData.VisualObject;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.george.transferfunctions.VisualObjectTransferFunction;

public class VisualObjectMediatorComponent extends PerceptMediatorComponent {

	@Override	
	protected PerceptBindingMediator<VisualObject, GroundedBelief> getMediator(String _toSA) {
		return PerceptBindingMediator.create(this,  _toSA, VisualObject.class,
				GroundedBelief.class, new VisualObjectTransferFunction(this));
	}

}
