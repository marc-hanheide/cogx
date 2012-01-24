package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;

public abstract class CASTResultWrapper<T>
implements CASTResult {

	private final T result;

	public CASTResultWrapper(T result) {
		this.result = result;
	}

	public final T getResult() {
		return result;
	}

	@Override
	public abstract WorkingMemoryAddress getRequestAddress();

}
