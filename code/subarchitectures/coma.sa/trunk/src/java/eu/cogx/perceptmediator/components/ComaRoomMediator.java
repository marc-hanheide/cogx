package eu.cogx.perceptmediator.components;

import java.util.Map;

import comadata.ComaRoom;

import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.DiscreteComaRoomTransferFunction;

public class ComaRoomMediator extends PerceptMediatorComponent {

	public static final double DEFAULT_HAS_PERSON_PROBABILITY = 0.5;

	public static final String HAS_PERSON_CONFIG = "--has-person";

	public static final String DISCRETE_CONFIG = "--discrete";

	private ComaRoomTransferFunction transferFunction = null;

	private double hasPersonProbability;

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * eu.cogx.perceptmediator.components.abstr.PerceptMediatorComponent#configure
	 * (java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {

		if (config.containsKey(HAS_PERSON_CONFIG)) {
			hasPersonProbability = Double.parseDouble(config
					.get(HAS_PERSON_CONFIG));
		} else {
			hasPersonProbability = DEFAULT_HAS_PERSON_PROBABILITY;
		}

		if (config.containsKey(DISCRETE_CONFIG))
			transferFunction = new DiscreteComaRoomTransferFunction(this,
					hasPersonProbability);
		else
			transferFunction = new ComaRoomTransferFunction(this,
					hasPersonProbability);

		println("using " + transferFunction.getClass().getSimpleName()
				+ " as transfer function.");
		/*
		 * make sure the super class is called at the end, because it calls
		 * getMediator(), which uses transferFunction, which was only set now!
		 */
		super.configure(config);
	}

	@Override
	protected PerceptBindingMediator<ComaRoom, GroundedBelief> getMediator(
			String _toSA) {
		return PerceptBindingMediator.create(this, _toSA, ComaRoom.class,
				GroundedBelief.class, transferFunction);
	}

}
