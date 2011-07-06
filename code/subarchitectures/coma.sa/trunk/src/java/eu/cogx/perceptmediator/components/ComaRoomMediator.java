package eu.cogx.perceptmediator.components;

import java.util.Map;

import comadata.ComaRoom;

import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.DiscreteComaRoomTransferFunction;

public class ComaRoomMediator extends PerceptMediatorComponent {

	private static final String DISCRETE_CONFIG = "--discrete";
	private ComaRoomTransferFunction transferFunction = null;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent#configure
	 * (java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {

		if (config.containsKey(DISCRETE_CONFIG))
			transferFunction = new DiscreteComaRoomTransferFunction(this);
		else
			transferFunction = new ComaRoomTransferFunction(this);

		println("using " + transferFunction.getClass().getSimpleName()
				+ " as transfer function.");
		/*
		 * make sure the super class is called at the end, because it calls
		 * getMediator(), which uses transferFunction, which was only set now!
		 */
		super.configure(config);
	}

	@Override
	protected PerceptBindingMediator<ComaRoom, GroundedBelief> getMediator() {
		return PerceptBindingMediator.create(this, ComaRoom.class,
				GroundedBelief.class, transferFunction);
	}

}
