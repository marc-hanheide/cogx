/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.conversion;

import java.util.Date;

import elm.event.*;
import celm.autogen.*;

/**
 * Converts between different data structures used for event times.
 */
public class EventTimeConverter {

	public static CELMEventTime toCEventTime(Date begin, Date end) {
		return toCEventTime(begin.getTime(), end.getTime());
	}

	public static CELMEventTime toCEventTime(long begin, long end) {
		return new CELMEventTime(new CELMEventTimestamp(begin),
				new CELMEventTimestamp(end));
	}

	public static CELMEventTime toCEventTime(EventTime e) {
		return toCEventTime(e.getMicroTimeBegin(), e.getMicroTimeEnd());
	}

	public static EventTime toEventTime(CELMEventTime c) {
		if (c == null) {
			return null;
		} else {
			return new EventTime(c.begin.milliseconds, c.end.milliseconds);
		}
	}

}
