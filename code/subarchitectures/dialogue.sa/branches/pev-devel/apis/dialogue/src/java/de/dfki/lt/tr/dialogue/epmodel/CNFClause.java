package de.dfki.lt.tr.dialogue.epmodel;

import java.util.ArrayList;
import java.util.List;

public class CNFClause<T> {

	private final List<Literal<T>> literals;

	public CNFClause() {
		literals = new ArrayList<Literal<T>>();
	}

	public int size() {
		return literals.size();
	}

}
