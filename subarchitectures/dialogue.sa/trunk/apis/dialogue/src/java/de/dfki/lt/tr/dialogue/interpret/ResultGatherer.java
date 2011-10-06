package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class ResultGatherer<T extends CASTResult> {

	private final WorkingMemoryAddress wma;
	private final ResultCombinator<T> combinator;
	private final CountDownLatch latch;

	public ResultGatherer(WorkingMemoryAddress wma, ResultCombinator<T> combinator) {
		this.wma = wma;
		this.combinator = combinator;
		latch = new CountDownLatch(1);
	}

	public synchronized void addResult(T result) {
		if (result.getRequestAddress().equals(wma)) {
			if (!isStabilized()) {
				combinator.addResult(result);
				if (combinator.resultsSufficient()) {
					stabilize();
				}
			}
		}
	}

	public T getResult() {
		return combinator.toResult();
	}

	private void stabilize() {
		latch.countDown();
	}

	synchronized public boolean isStabilized() {
		return latch.getCount() == 0;
	}

	public T ensureStabilization(long l, TimeUnit tu) {
		if (!isStabilized()) {
			try {
				latch.await(l, tu);
			}
			catch (InterruptedException ex) {
				// was interrupted, never mind!
			}
		}
		return getResult();
	}

}
