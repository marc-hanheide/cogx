package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
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

public class AgentAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "agent";

	private final WorkingMemoryAddress wma;
	private final String agName;

	public AgentAtom(WorkingMemoryAddress wma, String agName) {
		this.wma = wma;
		this.agName = agName;
	}

	public WorkingMemoryAddress getIntentionWMA() {
		return wma;
	}

	public String getAgentName() {
		return agName;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Intention
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					wma == null ? TermAtomFactory.var("IntID") : ConversionUtils.workingMemoryAddressToTerm(wma),
					agName == null ? TermAtomFactory.var("Ag") : TermAtomFactory.term(agName)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<AgentAtom> {

		@Override
		public AgentAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 2) {

				List<Term> args = matom.a.args;

				try {
					WorkingMemoryAddress wma = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(0));
					String agName = MatcherUtils.parseTermToString(args.get(1));

					return new AgentAtom(wma, agName);

				}
				catch (TermParsingException ex) {
					return null;
				}
			}

			return null;
		}

	}

	public static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

}
