package beliefmodels.builders;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.history.BinderHistory;

public abstract class AbstractBeliefBuilder {


	public static BinderHistory createHistory(Belief b) {
		
		String[] ancestors = new String[1];
		ancestors[0] = b.id;

		return new BinderHistory(ancestors, new String[0]);
	}
	
}
