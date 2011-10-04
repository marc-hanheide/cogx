package de.dfki.lt.tr.dialogue.interpret;

import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.ProcessingState;
import java.util.HashMap;
import java.util.Map;

public class InterpretedUserIntention
implements CASTProcessingResult, WellFormedTestable {

	private WorkingMemoryAddress wma;
	private final InterpretedIntention iint;

	public InterpretedUserIntention() {
		iint = newEmptyInterpretedIntention();
		wma = null;
	}

	@Override
	public void assertWellFormed() throws WellFormednessException {
		if (wma == null) {
			throw new WellFormednessException("WMA is null");
		}
		if (iint.agent == null || iint.agent.equals("")) {
			throw new WellFormednessException("agent is empty or null");
		}
	}

	public void setAddress(WorkingMemoryAddress wma) {
		this.wma = wma;
	}

	public void setAgent(String s) {
		iint.agent = s;
	}

	public void addStringContent(String key, String value) {
		iint.stringContent.put(key, value);
	}

	@Override
	public void commit(final WorkingMemoryWriterComponent component) throws SubarchitectureComponentException {
		component.getLogger().debug("about to write an InterpretedIntention to the WM");
		component.addToWorkingMemory(wma, iint);
	}

	public static InterpretedIntention newEmptyInterpretedIntention() {
		Map<String, String> stringContent = new HashMap<String, String>();
		Map<String, WorkingMemoryAddress> addressPointer = new HashMap<String, WorkingMemoryAddress>();
		String agent = "";
		float confidence = (float) 1.0;

		return new InterpretedIntention(stringContent, addressPointer, ProcessingState.READY, agent, confidence);
	}

	@Override
	public String toString() {
		return interpretedIntentionToString(iint, wma);
	}

	public static String interpretedIntentionToString(InterpretedIntention iint, WorkingMemoryAddress addr) {
		String s = "InterpretedIntention @ " + wmaToString(addr) + " {\n";
		s += "  state = " + iint.state.toString() + "\n";
		s += "  agent = " + iint.agent + "\n";
		s += "  confidence = " + iint.confidence + "\n";
		s += "  stringContent = {\n";
		for (String key : iint.stringContent.keySet()) {
			s += "    \"" + key + "\" -> \"" + iint.stringContent.get(key) + "\"\n";
		}
		s += "  }\n";
		s += "  pointerContent = {\n";
		for (String key : iint.addressContent.keySet()) {
			s += "    \"" + key + "\" -> \"" + wmaToString(iint.addressContent.get(key)) + "\"\n";
		}
		s += "  }\n";
		s += "}";
		return s;
	}

	public static String wmaToString(WorkingMemoryAddress wma) {
		if (wma != null) {
			return "[" + wma.id + "," + wma.subarchitecture + "]";
		}
		else {
			return "NULL";
		}
	}

}
