package comsys.processing.cca;

import Abducer.ModalisedFormula;
import Abducer.Predicate;

public interface Intention {

	public String getPredicateSymbol();

	public Predicate getPredicate();

	public boolean relatedTo(ModalisedFormula mf);
}
