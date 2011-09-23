package de.dfki.lt.tr.dialogue.ref.util;

public interface AbstractWMInterface<A, O> {

	public void add(A addr, O obj);

	public void overwrite(A addr, O obj);

	public void delete(A addr);

}
