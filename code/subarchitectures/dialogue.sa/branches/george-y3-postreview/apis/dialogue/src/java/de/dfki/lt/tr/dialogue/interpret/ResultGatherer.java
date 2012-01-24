package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public class ResultGatherer<T extends CASTResult> {

	private final WorkingMemoryAddress wma;
	private final ResultCombinator<T> combinator;
	private final CountDownLatch latch;
	private boolean satisfiedByResult;
	private int numResults;

	public ResultGatherer(WorkingMemoryAddress wma, ResultCombinator<T> combinator) {
		this.wma = wma;
		this.combinator = combinator;
		latch = new CountDownLatch(1);
		satisfiedByResult = false;
		numResults = 0;
	}

	public synchronized void addResult(T result) {
		if (result.getRequestAddress().equals(wma)) {
			if (!isStabilized()) {
				combinator.addResult(result);
				++numResults;
				if (combinator.resultsSufficient()) {
					stabilize();
				}
			}
		}
	}

	public T getResult() {
		return combinator.toResult();
	}

	private synchronized void stabilize() {
		latch.countDown();
		satisfiedByResult = true;
	}

	synchronized public boolean isStabilized() {
		return latch.getCount() == 0;
	}

	synchronized public boolean wasStabilizedByResult() {
		return satisfiedByResult;
	}

	public int getNumOfResults() {
		return numResults;
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
