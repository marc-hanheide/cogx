package beliefmodels.builders;

import java.util.LinkedList;
import java.util.List;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.history.BinderHistory;
import beliefmodels.arch.BeliefException;


public abstract class AbstractBeliefBuilder {


	public static BinderHistory createHistory(Belief b) throws BeliefException {
		if (b != null) { 
			List<String> ancestors = new LinkedList<String>();
			ancestors.add(b.id);
			return new BinderHistory(ancestors, new LinkedList<String>());
		} else { 
			throw new BeliefException("Error when creating belief history: cannot create history for null");
		}
	} 
	
}
