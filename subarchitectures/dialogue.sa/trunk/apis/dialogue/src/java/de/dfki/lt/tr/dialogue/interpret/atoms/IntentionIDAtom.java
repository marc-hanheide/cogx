package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;

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

				Term nomTerm = matom.a.args.get(0);
				Term wmaTerm = matom.a.args.get(1);

				String nominal = null;
				WorkingMemoryAddress wma = null;

				if (nomTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) nomTerm;
					if (isConstTerm(ft)) {
						nominal = ft.functor;
					}
					else {
						// unparseable!
						return null;
					}
				}

				if (wmaTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) wmaTerm;
					wma = ConversionUtils.termToWorkingMemoryAddress(wmaTerm);
					if (wma == null) {
						// unparseable!
						return null;
					}
				}
				
				return new IntentionIDAtom(nominal, wma);

			}

			return null;
		}

	}

	public static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

}
