package execution.util;

/**
 * A thread that sleeps then does something.
 * @author nah
 *
 */
public abstract class SleepyThread extends Thread {

	private final long m_sleepyTime;
	
	public SleepyThread(long _sleepyTime) {
		m_sleepyTime = _sleepyTime;
	}
	
	@Override
	public void run() {
		try {
			Thread.sleep(m_sleepyTime);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		
		doSomething();
	}

	protected abstract void doSomething();
	
}
