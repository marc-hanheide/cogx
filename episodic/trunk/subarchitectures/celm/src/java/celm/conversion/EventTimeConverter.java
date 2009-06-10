/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.conversion;

import java.util.Date;

import elm.event.*;
import celm.autogen.*;

/**
 *  Converts between different data structures used for event times. 
 */
public class EventTimeConverter {

    public static CELM_EventTime toCEventTime(Date begin, Date end) {

	return toCEventTime(begin.getTime(), end.getTime());
    }

    public static CELM_EventTime toCEventTime(long begin, long end) {

	return new CELM_EventTime(new CELM_EventTimestamp(begin),  
				  new CELM_EventTimestamp(end));
    }

    public static CELM_EventTime toCEventTime(EventTime e) {

	return toCEventTime(e.getMicroTimeBegin(), e.getMicroTimeEnd());
    }

    public static EventTime toEventTime(CELM_EventTime c) {

	return new EventTime(c.m_begin.m_milliseconds, c.m_end.m_milliseconds);
    }


}
