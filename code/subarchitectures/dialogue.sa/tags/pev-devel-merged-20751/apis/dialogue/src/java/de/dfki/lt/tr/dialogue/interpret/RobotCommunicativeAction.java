package de.dfki.lt.tr.dialogue.interpret;

import cast.SubarchitectureComponentException;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.LinkedList;
import java.util.List;

public class RobotCommunicativeAction
implements CASTProcessingResult, WellFormedTestable {

	private LogicalForm protoLF;

	public RobotCommunicativeAction() {
		protoLF = null;
	}

	@Override
	public void commit(WorkingMemoryWriterComponent component) throws SubarchitectureComponentException {
		WorkingMemoryAddress wma = new WorkingMemoryAddress(component.newDataID(), component.getSubarchitectureID());
		ContentPlanningGoal cpg = new ContentPlanningGoal(wma.id, protoLF, null);

		component.getLogger().info("will add the following protoLF to " + wmaToString(wma) + ":\n" + LFUtils.lfToString(protoLF));

		component.addToWorkingMemory(wma, cpg);
	}

	@Override
	public void assertWellFormed() throws WellFormednessException {
		if (protoLF == null) {
			throw new WellFormednessException("protoLF is null");
		}
	}

	public void setProtoLF(LogicalForm lf) {
		protoLF = lf;
	}

	public static List<ModalisedAtom> intentionToActToFacts(WorkingMemoryAddress wma, IntentionToAct actint) {
		List<ModalisedAtom> result = new LinkedList<ModalisedAtom>();

		for (String key : actint.stringContent.keySet()) {
			String value = actint.stringContent.get(key);

			result.add(TermAtomFactory.modalisedAtom(new Modality[] {
					Modality.Truth,
					Modality.Intention
				},
				TermAtomFactory.atom("string_content", new Term[] {
					ConversionUtils.workingMemoryAddressToTerm(wma),
					TermAtomFactory.term(key),
					TermAtomFactory.term(value)
				})));
		}

		for (String key : actint.addressContent.keySet()) {
			WorkingMemoryAddress value = actint.addressContent.get(key);

			result.add(TermAtomFactory.modalisedAtom(new Modality[] {
					Modality.Truth,
					Modality.Intention
				},
				TermAtomFactory.atom("address_content", new Term[] {
					ConversionUtils.workingMemoryAddressToTerm(wma),
					TermAtomFactory.term(key),
					ConversionUtils.workingMemoryAddressToTerm(value)
				})));
		}

		return result;
	}

	@Override
	public String toString() {
		String s = "(RobotCommunicativeAction\n";
		s += "  protoLF = " + (protoLF != null ? LFUtils.lfToString(protoLF) : "NULL") + "\n";
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
