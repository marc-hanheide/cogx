package de.dfki.lt.tr.dialogue.epmodel;

public class Literal<T> {

	private final Polarity pol;
	private final T atom;

	public Literal(T atom, Polarity pol) {
		this.atom = atom;
		this.pol = pol;
	}

	public Polarity getPolarity() {
		return pol;
	}

	public T getAtom() {
		return atom;
	}

}
