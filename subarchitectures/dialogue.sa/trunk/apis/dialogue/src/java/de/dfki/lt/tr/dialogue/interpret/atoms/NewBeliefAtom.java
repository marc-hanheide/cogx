package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;

public class NewBeliefAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "new_belief";

	private final WorkingMemoryAddress intentionAddr;
	private final WorkingMemoryAddress beliefAddr;
	private final EpistemicStatus epst;

	public NewBeliefAtom(WorkingMemoryAddress intentionAddr, WorkingMemoryAddress beliefAddr, EpistemicStatus epst) {
		this.intentionAddr = intentionAddr;
		this.beliefAddr = beliefAddr;
		this.epst = epst;
	}

	public WorkingMemoryAddress getIntentionAddress() {
		return intentionAddr;
	}

	public WorkingMemoryAddress getBeliefAddress() {
		return beliefAddr;
	}

	public EpistemicStatus getEpistemicStatus() {
		return epst;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Understanding
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					intentionAddr == null ? TermAtomFactory.var("IntentionAddr") : ConversionUtils.workingMemoryAddressToTerm(intentionAddr),
					beliefAddr == null ? TermAtomFactory.var("BeliefAddr") : ConversionUtils.workingMemoryAddressToTerm(beliefAddr),
					epst == null ? TermAtomFactory.var("EpSt") : ConversionUtils.epistemicStatusToTerm(epst)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<NewBeliefAtom> {

		@Override
		public NewBeliefAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 3) {

				Term intentionAddrTerm = matom.a.args.get(1);
				Term beliefAddrTerm = matom.a.args.get(1);
				Term epstTerm = matom.a.args.get(2);

				WorkingMemoryAddress intentionAddr = null;
				WorkingMemoryAddress beliefAddr = null;
				EpistemicStatus epst = null;

				if (intentionAddrTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) intentionAddrTerm;
					intentionAddr = ConversionUtils.termToWorkingMemoryAddress(intentionAddrTerm);
					if (intentionAddr == null) {
						// unparseable!
						return null;
					}
				}

				if (beliefAddrTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) beliefAddrTerm;
					beliefAddr = ConversionUtils.termToWorkingMemoryAddress(beliefAddrTerm);
					if (beliefAddr == null) {
						// unparseable!
						return null;
					}
				}

				if (epstTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) epstTerm;
					epst = ConversionUtils.termToEpistemicStatus(ft);
					if (epst == null) {
						// unparseable!
						return null;
					}
				}

				return new NewBeliefAtom(intentionAddr, beliefAddr, epst);

			}

			return null;
		}

	}

}