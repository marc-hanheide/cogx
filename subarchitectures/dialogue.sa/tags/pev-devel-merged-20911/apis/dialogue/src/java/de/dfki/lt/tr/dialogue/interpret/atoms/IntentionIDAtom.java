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

public class IntentionIDAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "intention";

	private final String nominal;
	private final WorkingMemoryAddress wma;

	public IntentionIDAtom(String nominal, WorkingMemoryAddress wma) {
		this.nominal = nominal;
		this.wma = wma;
	}

	public String getNominal() {
		return nominal;
	}

	public WorkingMemoryAddress getAddress() {
		return wma;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Understanding,
					Modality.Truth
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					nominal == null ? TermAtomFactory.var("Nom") : TermAtomFactory.term(nominal),
					wma == null ? TermAtomFactory.var("WMA") : ConversionUtils.workingMemoryAddressToTerm(wma)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<IntentionIDAtom> {

		@Override
		public IntentionIDAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 2) {

				List<Term> args = matom.a.args;

				try {
					String nominal = MatcherUtils.parseTermToString(args.get(0));
					WorkingMemoryAddress wma = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(1));

					return new IntentionIDAtom(nominal, wma);
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
