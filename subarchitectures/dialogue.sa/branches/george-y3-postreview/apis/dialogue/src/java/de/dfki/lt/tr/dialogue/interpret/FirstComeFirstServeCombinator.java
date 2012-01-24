package de.dfki.lt.tr.dialogue.interpret;


public class FirstComeFirstServeCombinator<T extends CASTResult>
implements ResultCombinator<T> {

	private T result;

	public FirstComeFirstServeCombinator() {
		result = null;
	}

	@Override
	public void addResult(T result) {
		this.result = result;
	}

	@Override
	public T toResult() {
		return result;
	}

	@Override
	public boolean resultsSufficient() {
		return result != null;
	}
	
}