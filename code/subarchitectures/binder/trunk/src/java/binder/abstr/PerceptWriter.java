package binder.abstr;

import binder.autogen.beliefs.PerceptBelief;
import cast.AlreadyExistsOnWMException;
import cast.architecture.ManagedComponent;

public class PerceptWriter extends ManagedComponent {

	
	public void insertPerceptInWM (PerceptBelief b) {
		try {
			addToWorkingMemory(b.id, b);
		}
		catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}
	}
}
