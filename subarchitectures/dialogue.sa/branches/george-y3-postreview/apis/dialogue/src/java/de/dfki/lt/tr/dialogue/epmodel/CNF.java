package de.dfki.lt.tr.dialogue.epmodel;

import java.util.ArrayList;
import java.util.List;

public class CNF<T> {

	private final List<CNFClause<T>> clauses;

	public CNF() {
		clauses = new ArrayList<CNFClause<T>>();
	}

	public int size() {
		return clauses.size();
	}

}
