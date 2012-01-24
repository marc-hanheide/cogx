package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
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

public class BeliefContentAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "belief_content";

	private final WorkingMemoryAddress beliefAddr;
	private final String key;
	private final dFormula value;

	public BeliefContentAtom(WorkingMemoryAddress wma, String key, dFormula value) {
		this.beliefAddr = wma;
		this.key = key;
		this.value = value;
	}

	public WorkingMemoryAddress getBeliefAddress() {
		return beliefAddr;
	}

	public String getKey() {
		return key;
	}

	public dFormula getValue() {
		return value;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Intention
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					beliefAddr == null ? TermAtomFactory.var("BeliefID") : ConversionUtils.workingMemoryAddressToTerm(beliefAddr),
					key == null ? TermAtomFactory.var("Key") : TermAtomFactory.term(key),
					value == null ? TermAtomFactory.var("Formula") : ConversionUtils.stateFormulaToTerm(value)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<BeliefContentAtom> {

		@Override
		public BeliefContentAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 3) {

				List<Term> args = matom.a.args;

				try {
					WorkingMemoryAddress wma = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(0));
					String key = MatcherUtils.parseTermToString(args.get(1));
					dFormula value = MatcherUtils.parseTermToDFormula(args.get(2));

					return new BeliefContentAtom(wma, key, value);
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