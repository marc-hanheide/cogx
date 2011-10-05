package de.dfki.lt.tr.dialogue.interpret;

import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.ProcessingState;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import java.util.HashMap;
import java.util.Map;

public class InterpretedUserIntention
implements CASTProcessingResult, WellFormedTestable {

	private WorkingMemoryAddress wma;
	private final InterpretedIntention iint;
	private final Map<WorkingMemoryAddress, dBelief> newBeliefs;

	public InterpretedUserIntention() {
		iint = newEmptyInterpretedIntention();
		wma = null;
		newBeliefs = new HashMap<WorkingMemoryAddress, dBelief>();
	}

	@Override
	public void assertWellFormed() throws WellFormednessException {
		if (wma == null) {
			throw new WellFormednessException("WMA is null");
		}
		if (iint.agent == null || iint.agent.equals("")) {
			throw new WellFormednessException("agent is empty or null");
		}
		for (dBelief b : newBeliefs.values()) {
			assertBeliefWellFormed(b);
		}
	}

	private void assertBeliefWellFormed(dBelief b) throws WellFormednessException {
		if (b == null) {
			throw new WellFormednessException("belief is null");
		}
		if (b.estatus == null) {
			throw new WellFormednessException("belief epistemic status is null");
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

	public void addAddressContent(String key, WorkingMemoryAddress value) {
		iint.addressContent.put(key, value);
	}

	public void addBeliefContent(WorkingMemoryAddress ref, String key, dFormula value) {
		dBelief bel = getBelief(ref);
		ConversionUtils.addFeature(bel, key, value);
	}

	public void setBeliefEpistemicStatus(WorkingMemoryAddress ref, EpistemicStatus epst) {
		dBelief bel = getBelief(ref);
		bel.estatus = epst;
	}

	protected dBelief getBelief(WorkingMemoryAddress ref) {
		dBelief bel = newBeliefs.get(ref);
		if (bel == null) {
			// create a new belief
			bel = ConversionUtils.newEmptyCondIndepDistribBelief(ref.id, null);
			newBeliefs.put(ref, bel);
		}
		return bel;
	}

	@Override
	public void commit(final WorkingMemoryWriterComponent component) throws SubarchitectureComponentException {
		component.getLogger().debug("about to write an InterpretedIntention to the WM");

		// determine which beliefs are to be added

		component.addToWorkingMemory(wma, iint);
	}

	public static InterpretedIntention newEmptyInterpretedIntention() {
		Map<String, String> stringContent = new HashMap<String, String>();
		Map<String, WorkingMemoryAddress> addressPointer = new HashMap<String, WorkingMemoryAddress>();
		String agent = "";
		float confidence = (float) 1.0;

		return new InterpretedIntention(stringContent, addressPointer, ProcessingState.READY, agent, confidence);
	}

//	public Set<WorkingMemoryAddress> referencedBeliefs() { }

	@Override
	public String toString() {
		String s = "[interpreted-user-intention struct:\n";
		s += wmaToString(wma) + " ... " + interpretedIntentionToString(iint, wma) + "\n";
		s += "\n";
		s += newBeliefs.size() + " beliefs need to be created:\n";
		for (WorkingMemoryAddress addr : newBeliefs.keySet()) {
			s += "\n";
			dBelief bel = newBeliefs.get(addr);
			s += wmaToString(addr) + " ... " + BeliefIntentionUtils.beliefToString(bel);
		}
		s += "\n";
		s += "]";
		return s;
	}

	public static String interpretedIntentionToString(InterpretedIntention iint, WorkingMemoryAddress addr) {
		String s = "(InterpretedIntention\n";
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
			s += "    \"" + key + "\" -> " + wmaToString(iint.addressContent.get(key)) + "\n";
		}
		s += "  }\n";
		s += ")";
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
