package eu.cogx.perceptmediator.components;

import VisionData.ViewConeMoveCostList;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;
import eu.cogx.perceptmediator.components.abstr.ReferringPerceptMediatorComponent;
import eu.cogx.perceptmediator.transferfunctions.ViewConeMoveCostsTransferFunction;

public class ViewConeMoveCostsMediatorComponent extends
		ReferringPerceptMediatorComponent<GroundedBelief> {

	public ViewConeMoveCostsMediatorComponent() {
		super(GroundedBelief.class);

	}

	@Override
	protected PerceptBindingMediator<ViewConeMoveCostList, GroundedBelief> getMediator(
			String _toSA) {
		return PerceptBindingMediator.create(this, _toSA, ViewConeMoveCostList.class,
				GroundedBelief.class, new ViewConeMoveCostsTransferFunction(this, allBeliefs));
	}

}
