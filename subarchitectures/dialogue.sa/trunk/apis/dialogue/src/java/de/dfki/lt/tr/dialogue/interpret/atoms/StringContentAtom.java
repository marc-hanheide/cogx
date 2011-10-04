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

public class StringContentAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "string_content";

	private final WorkingMemoryAddress wma;
	private final String key;
	private final String value;

	public StringContentAtom(WorkingMemoryAddress wma, String key, String value) {
		this.wma = wma;
		this.key = key;
		this.value = value;
	}

	public WorkingMemoryAddress getIntentionWMA() {
		return wma;
	}

	public String getKey() {
		return key;
	}

	public String getValue() {
		return value;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		return TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Intention
				},
				TermAtomFactory.atom(PRED_SYMBOL, new Term[] {
					wma == null ? TermAtomFactory.var("IntID") : ConversionUtils.workingMemoryAddressToTerm(wma),
					key == null ? TermAtomFactory.var("Key") : TermAtomFactory.term(key),
					value == null ? TermAtomFactory.var("Value") : TermAtomFactory.term(value)
				}));
	}

	public static class Matcher implements ModalisedAtomMatcher<StringContentAtom> {

		@Override
		public StringContentAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.equals(PRED_SYMBOL)
					&& matom.a.args.size() == 3) {

				Term wmaTerm = matom.a.args.get(0);
				Term keyTerm = matom.a.args.get(1);
				Term valueTerm = matom.a.args.get(2);

				WorkingMemoryAddress wma = null;
				String key = null;
				String value = null;

				if (wmaTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) wmaTerm;
					wma = ConversionUtils.termToWorkingMemoryAddress(wmaTerm);
					if (wma == null) {
						// unparseable!
						return null;
					}
				}

				if (keyTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) keyTerm;
					if (isConstTerm(ft)) {
						key = ft.functor;
					}
					else {
						// unparseable!
						return null;
					}
				}

				if (valueTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) valueTerm;
					if (isConstTerm(ft)) {
						value = ft.functor;
					}
					else {
						// unparseable!
						return null;
					}
				}

				return new StringContentAtom(wma, key, value);

			}

			return null;
		}

	}

	public static boolean isConstTerm(FunctionTerm ft) {
		return ft.args.isEmpty();
	}

}