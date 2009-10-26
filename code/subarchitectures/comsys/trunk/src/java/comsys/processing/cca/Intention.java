package comsys.processing.cca;

import beliefmodels.adl.Agent;
import Abducer.ModalisedFormula;
import Abducer.Predicate;

public interface Intention {

	public Predicate toPredicate();

	public Agent sourceAgent();

	public ContextUpdate doUpdate(ProofStack stack);
}
