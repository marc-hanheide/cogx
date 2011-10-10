package de.dfki.lt.tr.dialogue.interpret;

public class MaximumReadingsTerminationCondition
implements TerminationCondition {

	private final int limit;

	public MaximumReadingsTerminationCondition(int limit) {
		this.limit = limit;
	}

	@Override
	public boolean reached() {
		throw new UnsupportedOperationException("Not supported yet.");
	}
	
}
