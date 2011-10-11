package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import java.util.List;

public interface ProofInterpreter<T> {

	public T interpret(List<ModalisedAtom> matoms, double cost);

}
