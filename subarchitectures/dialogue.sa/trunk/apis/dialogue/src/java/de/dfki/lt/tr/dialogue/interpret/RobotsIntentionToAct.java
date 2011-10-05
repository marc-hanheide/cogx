package de.dfki.lt.tr.dialogue.interpret;

import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryWriterComponent;

public class RobotsIntentionToAct
implements CASTProcessingResult, WellFormedTestable {

	@Override
	public void commit(WorkingMemoryWriterComponent component) throws SubarchitectureComponentException {
		throw new UnsupportedOperationException("Not supported yet.");
	}

	@Override
	public void assertWellFormed() throws WellFormednessException {
		throw new UnsupportedOperationException("Not supported yet.");
	}
	
}
