package de.dfki.lt.tr.dialogue.interpret;

public class MaximumReadingsTerminationCondition<T>
implements TerminationCondition<T> {

	private final int limit;

	public MaximumReadingsTerminationCondition(int limit) {
		this.limit = limit;
	}

	@Override
	public boolean reached(PartialInterpretation<T, ?> pinpr) {
		return pinpr.getInterpretations().size() >= limit;
	}
	
}
