package de.dfki.lt.tr.dialogue.epmodel;

public interface IntentionIface {

	public CNF<StateFormula> getPreconditions();

	public CNF<StateFormula> getPostconditions();

}
