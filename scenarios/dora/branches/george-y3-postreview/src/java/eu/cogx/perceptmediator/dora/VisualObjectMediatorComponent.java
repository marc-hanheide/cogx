package eu.cogx.perceptmediator.dora;

import VisionData.VisualObject;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;

public class VisualObjectMediatorComponent extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

    VisualObjectTransferFunction tf = null;

	public VisualObjectMediatorComponent() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<VisualObject, PerceptBelief> getMediator(String _toSA) {
        tf = new VisualObjectTransferFunction(this, allBeliefs);
		return PerceptBindingMediator.create(this, _toSA, VisualObject.class,
				PerceptBelief.class, tf);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
        tf.start();
        super.start();
    }

}
