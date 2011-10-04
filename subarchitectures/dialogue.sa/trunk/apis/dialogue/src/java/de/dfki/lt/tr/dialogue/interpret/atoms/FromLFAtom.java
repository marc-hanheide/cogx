package de.dfki.lt.tr.dialogue.interpret.atoms;

import de.dfki.lt.tr.dialogue.interpret.InterpretableAtom;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.proof.ModalisedAtomMatcher;

public class FromLFAtom
implements InterpretableAtom {

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
			if (matom.a.predSym.contains("from_logical_form") && matom.a.args.size() == 3
					&& matom.a.args.get(2) instanceof FunctionTerm) {

				String nominal = ((FunctionTerm) matom.a.args.get(2)).functor;
				return new FromLFAtom(nominal);
			}
			return null;
		}
		
	}

}
