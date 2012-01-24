package de.dfki.lt.tr.dialogue.interpret.atoms;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.dialogue.interpret.MatcherUtils;
import de.dfki.lt.tr.dialogue.interpret.TermParsingException;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import java.util.List;

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

				List<Term> args = matom.a.args;

				try {
					WorkingMemoryAddress wma = MatcherUtils.parseTermToWorkingMemoryAddress(args.get(2));
					return new FromIntentionAtom(wma);
				}
				catch (TermParsingException ex) {
					return null;
				}

			}
			return null;
		}
		
	}

}
