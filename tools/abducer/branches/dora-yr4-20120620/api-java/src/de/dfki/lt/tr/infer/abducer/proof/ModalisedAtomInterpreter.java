package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;

public abstract class ModalisedAtomInterpreter<MatchT, ResultT> {

	private final ModalisedAtomMatcher<MatchT> matcher;

	public ModalisedAtomInterpreter(ModalisedAtomMatcher<MatchT> matcher) {
		this.matcher = matcher;
	}

	public final ResultT matchAndActOn(ModalisedAtom matom) {
		MatchT matchResult = matcher.match(matom);
		if (matchResult != null) {
			return actOn(matchResult);
		}
		else {
			return null;
		}
	}

	public abstract ResultT actOn(MatchT matchResult);

}
