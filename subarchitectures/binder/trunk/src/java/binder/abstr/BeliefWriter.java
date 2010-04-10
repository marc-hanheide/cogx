package binder.abstr;

import beliefmodels.autogen.beliefs.Belief;
import cast.AlreadyExistsOnWMException;
import cast.architecture.ManagedComponent;

public class BeliefWriter extends ManagedComponent {

	
	public void insertBeliefInWM (Belief b) {
		try {
			addToWorkingMemory(b.id, b);
		}
		catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}
}
