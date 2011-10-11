package de.dfki.lt.tr.dialogue.interpret;

import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
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

	public WorkingMemoryAddress getAddress() {
		return wma;
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

		// determine which beliefs are to be added
		for (WorkingMemoryAddress addr : newBeliefs.keySet()) {
			dBelief bel = newBeliefs.get(addr);
			component.getLogger().debug("adding a dBelief to " + wmaToString(addr));
			component.addToWorkingMemory(addr, bel);
		}

		component.getLogger().debug("adding an InterpretedIntention to " + wmaToString(wma));
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
		s += wmaToString(wma) + " ... " + interpretedIntentionToString(iint) + "\n";
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

	public static String interpretedIntentionToString(InterpretedIntention iint) {
		String s = "(InterpretedIntention\n";
		s += "  state = " + iint.state.toString() + "\n";
		s += "  agent = " + iint.agent + "\n";
		s += "  confidence = " + iint.confidence + "\n";
		s += baseIntentionToString(iint, "  ") + "\n";
		s += ")";
		return s;
	}

	public static String baseIntentionToString(BaseIntention bint, String indent) {
		String s = indent + "(\n";
		s += indent + "  stringContent = {\n";
		for (String key : bint.stringContent.keySet()) {
			s += indent + "    \"" + key + "\" -> \"" + bint.stringContent.get(key) + "\"\n";
		}
		s += indent + "  }\n";
		s += indent + "  pointerContent = {\n";
		for (String key : bint.addressContent.keySet()) {
			s += indent + "    \"" + key + "\" -> " + wmaToString(bint.addressContent.get(key)) + "\n";
		}
		s += indent + "  }\n";
		s += indent + ")";
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

	public Map<WorkingMemoryAddress, dBelief> toBeliefs(WMAddressTranslator translator) {
		Map<WorkingMemoryAddress, dBelief> bels = new HashMap<WorkingMemoryAddress, dBelief>();

		for (WorkingMemoryAddress addr : newBeliefs.keySet()) {
			dBelief bel = newBeliefs.get(addr);
			bels.put(translator.translate(addr), bel);
		}

		return bels;
	}

	public InterpretedIntention toIntention(WMAddressTranslator translator) {
		InterpretedIntention intention = cloneIntention(iint);

		Map<String, WorkingMemoryAddress> newAddressContent = new HashMap<String, WorkingMemoryAddress>();
		for (String key : iint.addressContent.keySet()) {
			WorkingMemoryAddress addr = iint.addressContent.get(key);
			if (newBeliefs.containsKey(addr)) {
				// need to translate it
				addr = translator.translate(addr);
			}
			newAddressContent.put(key, addr);
		}
		intention.addressContent = newAddressContent;

		return intention;
	}

	public static InterpretedIntention cloneIntention(InterpretedIntention iint) {
		InterpretedIntention newInt = newEmptyInterpretedIntention();
		newInt.agent = iint.agent;
		newInt.confidence = iint.confidence;
		newInt.state = iint.state;
		newInt.stringContent.putAll(iint.stringContent);
		newInt.addressContent.putAll(iint.addressContent);

		return newInt;
	}

	public void setConfidence(double confidence) {
		iint.confidence = (float) confidence;
	}

}
