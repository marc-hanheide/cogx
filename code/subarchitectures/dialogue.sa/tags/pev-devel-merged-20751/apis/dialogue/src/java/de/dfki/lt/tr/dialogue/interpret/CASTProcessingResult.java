package de.dfki.lt.tr.dialogue.interpret;

import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryWriterComponent;

public interface CASTProcessingResult {

	public abstract void commit(WorkingMemoryWriterComponent component) throws SubarchitectureComponentException;

}
