package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.dialogue.interpret.MatcherUtils;
import de.dfki.lt.tr.dialogue.interpret.TermParsingException;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.List;

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

				List<Term> args = matom.a.args;

				try {
					WorkingMemoryAddress intentionAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(0));
					WorkingMemoryAddress beliefAddr = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(1));
					EpistemicStatus epst = MatcherUtils.parseTermToEpistemicObject(args.get(2));

					return new NewBeliefAtom(intentionAddr, beliefAddr, epst);
				}
				catch (TermParsingException ex) {
					return null;
				}
			}

			return null;
		}

	}

}