/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.conversion;

import java.util.Date;

import Ice.Communicator;
import Ice.Identity;
import Ice.ObjectPrx;
import cast.cdl.CASTTime;
import cast.cdl.CPPSERVERPORT;
import cast.interfaces.TimeServerPrx;
import cast.interfaces.TimeServerPrxHelper;

public class CASTTimeConverter {

	/**
	 * Threshold in seconds (!) for guessing whether a given CASTTime is
	 * relative to startup time (i.e. t = (0, 0) at startup) or relative to
	 * Epoch (as in C calls to time() or gettimeofday()). <br>
	 * This is (currently) needed since sometimes Nav.SA reports RobotPoses with
	 * startup-relative time (in stage-simulation) or Epoch-based (real robot
	 * runs). <br>
	 * The specified time corresponds to Sat, 01 Jan 2000 00:00:00 GMT.
	 */
	public static final long guessingThreshold = 946684800;

	private static long offset = -1;

	private static TimeServerPrx m_staticTimeServer;

	/**
	 * This resolves and stores a time server. This will be added to CASTUtils
	 * in the next release.
	 * 
	 * @return
	 */
	public static final TimeServerPrx getTimeServer() {
		if (m_staticTimeServer == null) {
			String host = "localhost";
			int port = CPPSERVERPORT.value;
			Communicator ic = Ice.Util.initialize();
			Identity id = new Identity("TimeServer", "TimeServer");
			ObjectPrx base = ic.stringToProxy(ic.identityToString(id)
					+ ":default -h " + host + " -p " + port);
			m_staticTimeServer = TimeServerPrxHelper.checkedCast(base);
		}
		return m_staticTimeServer;
	}

	/**
	 * Convert the given CASTTime bt to a Java Date object, guessing whether bt
	 * is relative to Epoch or CAST startup time.
	 */
	public static Date toJavaDate(CASTTime bt) {
		return new Date(toMillisecondsSinceEpochTime(bt));
	}

	/**
	 * Convert the given CASTTime bt to a long integer (containing the number of
	 * milliseconds since Epoch), guessing whether bt is relative to Epoch or
	 * CAST startup time.
	 */
	public static long toMillisecondsSinceEpochTime(CASTTime bt) {

		if (bt.s < guessingThreshold)
			return toMillisecondsSinceEpochTime(bt, getCASTTimeOffset());
		else
			return toMilliseconds(bt);
	}

	/**
	 * Estimate how many milliseconds have passed from Epoch (Jan 1970) until
	 * system startup (CASTTime (0 , 0)).
	 */
	public static long computeCASTTimeOffset() {

		long t1 = System.currentTimeMillis();
		CASTTime bt = getTimeServer().getCASTTime();
		
		long t2 = System.currentTimeMillis();

		long tm = (t1 + t2) / 2;
		return (tm - toMilliseconds(bt));
	}

	public static void setOffset(long newOffsetInMilliseconds) {

		offset = newOffsetInMilliseconds;
	}

	/**
	 * Get the precomputed CASTTimeOffset or compute it.
	 * 
	 * @see #computeCASTTimeOffset()
	 */
	public static long getCASTTimeOffset() {

		if (offset == -1)
			offset = computeCASTTimeOffset();
		return offset;
	}

	/**
	 * Convert the given CASTTime to milliseconds without any consideration of
	 * an offset.
	 */
	public static long toMilliseconds(CASTTime bt) {

		return (long) bt.s * 1000 + (long) bt.us / 1000;
	}

	/**
	 * Convert the given CASTTime to milliseconds with a CASTTime offset as
	 * specified.
	 */
	public static long toMillisecondsSinceEpochTime(CASTTime bt, long offset) {

		return (toMilliseconds(bt) + offset);
	}

}
