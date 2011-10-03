package de.dfki.lt.tr.dialogue.interpret;

import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.ProcessingState;
import java.util.HashMap;
import java.util.Map;

public class InterpretedUserIntention
implements CASTProcessingResult {

	private final InterpretedIntention iint;

	public InterpretedUserIntention() {
		iint = newEmptyInterpretedIntention();
	}

	public boolean isWellFormed() {
		if (!iint.agent.equals("")) {
			return false;
		}
		return true;
	}

	@Override
	public void commit(final WorkingMemoryWriterComponent component) throws SubarchitectureComponentException {
		component.getLogger().debug("about to write an InterpretedIntention to the WM");
		component.addToWorkingMemory(component.newDataID(), iint);
	}

	public static InterpretedIntention newEmptyInterpretedIntention() {
		Map<String, String> stringContent = new HashMap<String, String>();
		Map<String, WorkingMemoryPointer> pointerContent = new HashMap<String, WorkingMemoryPointer>();
		String agent = "";
		float confidence = (float) 1.0;

		return new InterpretedIntention(stringContent, pointerContent, ProcessingState.READY, agent, confidence);
	}

}
