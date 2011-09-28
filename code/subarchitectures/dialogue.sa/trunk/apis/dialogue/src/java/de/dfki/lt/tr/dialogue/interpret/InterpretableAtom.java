package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;

public interface InterpretableAtom {

	public ModalisedAtom toModalisedAtom();

}
