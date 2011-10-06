package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

public class ResultGatherer<T extends CASTResult> {

	private final WorkingMemoryAddress wma;
	private final ResultFuture future;

	public ResultGatherer(WorkingMemoryAddress wma) {
		this.wma = wma;
		future = new ResultFuture();
	}

	public synchronized void addResult(T result) {
		if (!future.isDone()) {
			future.setResult(result);
			future.finish();
		}
	}

//	public void gathered() {
//		future.finished();
//	}

	public Future<T> getResultFuture() {
		return future;
	}

	private class ResultFuture implements Future<T> {

		private T result = null;
		private CountDownLatch latch = new CountDownLatch(1);

		public void finish() {
			latch.countDown();
		}

		public void setResult(T result) {
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
		public T get() throws InterruptedException, ExecutionException {
			latch.await();
			return result;
		}

		@Override
		public T get(long l, TimeUnit tu) throws InterruptedException, ExecutionException, TimeoutException {
			latch.await(l, tu);
			return result;
		}
		
	}

}
