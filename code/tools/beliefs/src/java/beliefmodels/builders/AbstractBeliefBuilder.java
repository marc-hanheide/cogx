package beliefmodels.builders;

import java.util.LinkedList;
import java.util.List;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.history.BinderHistory;

public abstract class AbstractBeliefBuilder {


	public static BinderHistory createHistory(Belief b) {
		
		List<String> ancestors = new LinkedList<String>();
		ancestors.add(b.id);

		return new BinderHistory(ancestors, new LinkedList<String>());
	}
	
}
