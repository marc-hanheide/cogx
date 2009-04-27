/**
 * 
 */
package cast.server;

import Ice.Current;
import cast.cdl.CASTTime;
import cast.core.CASTComponent;
import cast.interfaces._ComponentManagerDisp;
import cast.interfaces._ComponentManagerOperations;

/**
 * @author nah
 * 
 */
public class CASTComponentManager extends _ComponentManagerDisp {

	final static private long NANOS_PER_SECOND = 1000000000;
	final static private long NANOS_PER_MICROSECOND = 1000;

	private long m_startupNanos;

	/**
	 * 
	 * @param _id
	 *            The id of Ice server
	 */
	public CASTComponentManager() {
//		System.out.println("CASTComponentManager.CASTComponentManager()");
		m_startupNanos = System.nanoTime();
	}

	public final CASTTime getCASTTime(Current _current) {
		long elapsedNanos = System.nanoTime() - m_startupNanos;
		long elapsedSeconds = elapsedNanos / NANOS_PER_SECOND;
		long remainderNanos = elapsedNanos % NANOS_PER_SECOND;
		long remainderMicros = remainderNanos / NANOS_PER_MICROSECOND;
		return new CASTTime(elapsedSeconds, remainderMicros);
	}

	

}
