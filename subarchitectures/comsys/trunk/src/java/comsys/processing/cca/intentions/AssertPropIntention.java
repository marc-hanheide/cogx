package comsys.processing.cca.intentions;

import beliefmodels.adl.Agent;
import Abducer.ModalisedFormula;
import Abducer.Predicate;
import Abducer.Term;
import comsys.processing.cca.ContextUpdate;
import comsys.processing.cca.Intention;
import comsys.processing.cca.ProofStack;

public class AssertPropIntention
		implements Intention {

	public Predicate toPredicate() {
		Predicate p = new Predicate();
		p.predSym = "assert_prop";
		p.args = new Term[0];
		return p;
	}

	public Agent sourceAgent() {
		return null;
	}

	public ContextUpdate doUpdate(ProofStack stack) {
		return null;
	}
	
}
