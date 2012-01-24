package eu.cogx.perceptmediator.george.components;


import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.george.transferfunctions.RobotTransferFunction;
import execution.slice.Robot;

public class RobotMediatorComponent extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public RobotMediatorComponent() {
		super(GroundedBelief.class);
	}

	@Override
	protected PerceptBindingMediator<Robot, GroundedBelief> getMediator(String _toSA) {
		return PerceptBindingMediator.create(this, _toSA, Robot.class,
				GroundedBelief.class, new RobotTransferFunction(this,
						this.allBeliefs));
	}

}
