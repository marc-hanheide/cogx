package comsys.processing.cca;

import beliefmodels.adl.*;

public class VerifiableUpdate {

	public enum Consistency {
		Consistent,
		Inconsistent
	}

	public static Consistency consistent(BeliefModel bm, Belief assertion, Belief[] assumed) {
		System.err.println("TODO: VerifiableUpdate.consistent");
		return Consistency.Consistent;
	}
	
}
