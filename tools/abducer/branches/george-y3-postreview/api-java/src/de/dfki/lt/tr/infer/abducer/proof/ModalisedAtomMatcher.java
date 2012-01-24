package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;

public interface ModalisedAtomMatcher<T> {

	public T match(ModalisedAtom matom);

}
