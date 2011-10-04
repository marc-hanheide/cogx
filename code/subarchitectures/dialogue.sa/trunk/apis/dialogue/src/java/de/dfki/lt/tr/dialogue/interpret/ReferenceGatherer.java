package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class ReferenceGatherer {

	private final WorkingMemoryAddress wma;
	private final ReferenceResolutionRequest request;
	private final ResultFuture future;

	public ReferenceGatherer(WorkingMemoryAddress wma, ReferenceResolutionRequest request) {
		this.wma = wma;
		this.request = request;
		future = new ResultFuture();
	}

	public synchronized void addResult(ReferenceResolutionResult result) {
		if (!future.isDone()) {
			future.setResult(result);
			future.finish();
		}
	}

//	public void gathered() {
//		future.finished();
//	}

	public Future<ReferenceResolutionResult> getResultFuture() {
		return future;
	}

	private class ResultFuture implements Future<ReferenceResolutionResult> {

		private ReferenceResolutionResult result = null;
		private CountDownLatch latch = new CountDownLatch(1);

		public void finish() {
			latch.countDown();
		}

		public void setResult(ReferenceResolutionResult result) {
			this.result = result;
		}

		@Override
		public boolean cancel(boolean bln) {
			throw new UnsupportedOperationException("Not supported.");
		}

		@Override
		public boolean isCancelled() {
			throw new UnsupportedOperationException("Not supported.");
		}

		@Override
		public synchronized boolean isDone() {
			return latch.getCount() == 0;
		}

		@Override
		public ReferenceResolutionResult get() throws InterruptedException, ExecutionException {
			latch.await();
			return result;
		}

		@Override
		public ReferenceResolutionResult get(long l, TimeUnit tu) throws InterruptedException, ExecutionException, TimeoutException {
			latch.await(l, tu);
			return result;
		}
		
	}

}
