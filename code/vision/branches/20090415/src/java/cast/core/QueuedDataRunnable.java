/**
 * 
 */
package cast.core;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Semaphore;

/**
 * A runnable that takes data from one thread, then queues it and forwards it in
 * another thread.
 * 
 * @author nah
 * 
 */
public abstract class QueuedDataRunnable<QueuedData> extends ControlledRunnable {

	/**
	 * The stored change events.
	 */
	private Queue<QueuedData> m_queue;

	private Semaphore m_changeSemaphore;

	public QueuedDataRunnable() {
		m_changeSemaphore = new Semaphore(0, true);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Runnable#run()
	 */
	public void run() {

		QueuedData data;

		while (isRunning()) {

			while (m_queue != null && !m_queue.isEmpty()) {

				synchronized (m_queue) {
					data = m_queue.poll();
				}

				nextInQueue(data);

			}

			try {
				m_changeSemaphore.acquire();
			} catch (InterruptedException e) {
				e.printStackTrace();
				System.exit(1);
			}

		}

	}

	protected abstract void nextInQueue(QueuedData _data);

	public void queue(QueuedData _data) {

		if (m_queue == null) {
			m_queue = new ConcurrentLinkedQueue<QueuedData>();
		}

		synchronized (m_queue) {
			m_queue.add(_data);
		}
		m_changeSemaphore.release();
	}

}
