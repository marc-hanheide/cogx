package beliefmodels.builders;

import java.util.LinkedList;
import java.util.List;

import cast.cdl.WorkingMemoryAddress;

import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.arch.BeliefException;


public abstract class AbstractBeliefBuilder {

 
	public static CASTBeliefHistory createHistory(WorkingMemoryAddress previousBelief) throws BeliefException {
		if (previousBelief != null) { 
			List<WorkingMemoryAddress> ancestors = new LinkedList<WorkingMemoryAddress>();
			ancestors.add(previousBelief);
			return new CASTBeliefHistory(ancestors, new LinkedList<WorkingMemoryAddress>());
		} else { 
			throw new BeliefException("Error when creating belief history: cannot create history for null");
		}
	} 
	
}
