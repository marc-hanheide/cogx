package de.dfki.lt.tr.dialogue.interpret;

public interface TerminationCondition<T> {

	public boolean reached(PartialInterpretation<T, ?> pinpr);

}
