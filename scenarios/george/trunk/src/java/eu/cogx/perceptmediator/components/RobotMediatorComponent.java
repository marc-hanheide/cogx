package eu.cogx.perceptmediator.components;


import VisionData.ViewCone;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.RobotTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.ViewConeTransferFunction;
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
