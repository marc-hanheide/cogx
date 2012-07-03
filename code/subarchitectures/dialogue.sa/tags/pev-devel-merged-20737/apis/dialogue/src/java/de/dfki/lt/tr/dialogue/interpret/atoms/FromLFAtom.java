package de.dfki.lt.tr.dialogue.interpret.atoms;

import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.dialogue.interpret.MatcherUtils;
import de.dfki.lt.tr.dialogue.interpret.TermParsingException;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;
import java.util.List;

public class FromLFAtom
implements InterpretableAtom {

	public static final String PRED_SYMBOL = "from_logical_form";

	private final String nominal;

	public FromLFAtom(String nominal) {
		this.nominal = nominal;
	}

	public String getNominal() {
		return nominal;
	}

	@Override
	public ModalisedAtom toModalisedAtom() {
		throw new UnsupportedOperationException("Not supported yet.");
	}

	public static class Matcher implements ModalisedAtomMatcher<FromLFAtom> {

		@Override
		public FromLFAtom match(ModalisedAtom matom) {
			if (matom.a.predSym.contains(PRED_SYMBOL) && matom.a.args.size() == 3) {

				List<Term> args = matom.a.args;

				try {
					String nominal = MatcherUtils.parseTermToString(args.get(2));

					return new FromLFAtom(nominal);
				}
				catch (TermParsingException ex) {
					return null;
				}

			}
			return null;
		}
		
	}

}
