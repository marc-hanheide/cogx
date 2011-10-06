package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;

public class FromIntentionAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "from_intention";

	private final WorkingMemoryAddress wma;

	public FromIntentionAtom(WorkingMemoryAddress wma) {
		this.wma = wma;
	}

	public WorkingMemoryAddress getAddress() {
		return wma;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		throw new UnsupportedOperationException("Not supported yet.");
	}

	public static class Matcher implements ModalisedAtomMatcher<FromIntentionAtom> {

		@Override
		public FromIntentionAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.contains(PRED_SYMBOL) && matom.a.args.size() == 3) {

				Term wmaTerm = matom.a.args.get(2);

				WorkingMemoryAddress wma = null;

				if (wmaTerm instanceof FunctionTerm) {
					FunctionTerm ft = (FunctionTerm) wmaTerm;
					wma = ConversionUtils.termToWorkingMemoryAddress(wmaTerm);
					if (wma == null) {
						// unparseable!
						return null;
					}
				}

				return new FromIntentionAtom(wma);
			}
			return null;
		}
		
	}

}
