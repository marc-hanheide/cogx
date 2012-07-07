package castutils.time;

import org.apache.log4j.Logger;

/**
 * A class to make a loop run at a particular rate. The logic is that the loop
 * including the call to sleep will run at the value given in the constructor.
 * THe warning multiplier allows you to configure a warning to be printed if the
 * duration of the loop exceeds a multiple of the configured rate.
 * 
 * @author nah
 * 
 */
public class Rate {

	private final long m_loopDurationMs;
	private long m_lastSystemTimeMs;
	private final long m_warningTimeMs;

	private static final Logger logger = Logger
			.getLogger(castutils.time.Rate.class);

	public Rate(float _hertz, int _warningMultiplier) {
		m_loopDurationMs = (long) (1000 / _hertz);
		m_warningTimeMs = m_loopDurationMs * _warningMultiplier;
		m_lastSystemTimeMs = System.currentTimeMillis();
	}

	public Rate(float _hertz) {
		this(_hertz, 10);
	}

	public void sleep() {
		long currentSystemTime = System.currentTimeMillis();
		long lastLoopDuration = currentSystemTime - m_lastSystemTimeMs;
		// logger.info("last loop: " + lastLoopDuration);

		if (lastLoopDuration < m_loopDurationMs) {
			try {
				long sleepDuration = m_loopDurationMs - lastLoopDuration;
				// logger.info("sleeping for: " + sleepDuration);
				Thread.sleep(sleepDuration);
			} catch (InterruptedException e) {
				logger.warn(e.getMessage(), e);
			}
		} else if (lastLoopDuration > m_warningTimeMs) {
			logger.warn("Rate of loop exeeded configured warning threshold");
		}

		m_lastSystemTimeMs = currentSystemTime;

	}

	/**
	 * @param args
	 * @throws InterruptedException
	 */
	public static void main(String[] args) throws InterruptedException {

		Rate rate = new Rate(200);

		for (int i = 0; i < 50; i++) {
			rate.sleep();
			logger.info("doing stuff");
//			Thread.sleep((long) (Math.random() * 500));
		}

	}

}
