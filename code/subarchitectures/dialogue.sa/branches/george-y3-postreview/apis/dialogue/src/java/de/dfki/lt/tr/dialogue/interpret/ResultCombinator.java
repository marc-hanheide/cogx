package de.dfki.lt.tr.dialogue.interpret;

public interface ResultCombinator<T extends CASTResult> {

	public void addResult(T result);

	public T toResult();

	public boolean resultsSufficient();

}
