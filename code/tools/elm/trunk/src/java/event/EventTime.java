/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import java.util.Collection;
import java.util.Iterator;
import java.util.Date;
import java.sql.Timestamp;

public class EventTime {

    private long microTimeBegin 	= Long.MAX_VALUE;
    private long microTimeEnd		= Long.MIN_VALUE;
    
    public EventTime(long microTimeBegin, long microTimeEnd) {
        this.microTimeBegin = microTimeBegin;
        this.microTimeEnd   = microTimeEnd;
    }
    
    public EventTime(Date begin, Date end) {
        this.microTimeBegin = begin.getTime();
        this.microTimeEnd   = end.getTime();
    }

    public EventTime(EventTime[] subEventTimes) {

	for (int i = 0; i < subEventTimes.length; i++) {
	    addSubEventTime(subEventTimes[i]);
	}
    }
    

    public EventTime(Collection<Event> subEvents) {
	
	Iterator<Event> iterator = subEvents.iterator();

	while (iterator.hasNext())
	    addSubEventTime(iterator.next().getTime());

    }

    public EventTime(Event[] subEvents) {
	
	for (int i = 0; i < subEvents.length; i++)
	    addSubEventTime(subEvents[i].getTime());
    }
    
    protected void addSubEventTime(EventTime t) {
	long begin = t.getMicroTimeBegin();
	long end   = t.getMicroTimeEnd();
	if (begin < microTimeBegin)
	    microTimeBegin = begin;
	if (end > microTimeEnd)
	    microTimeEnd = end;
    }

    public long getMicroTimeBegin() {
	return microTimeBegin;
    }

    public long getMicroTimeEnd() {
	return microTimeEnd;
    }

    public Date getBegin() {
	// return new Date(microTimeBegin);
	return new Timestamp(microTimeBegin);
    }

    public Date getEnd() {
	// return new Date(microTimeEnd);
	return new Timestamp(microTimeEnd);
    }

    public double getBeginInSeconds() {
	return microTimeBegin * 1e-3;
    }
    public double getEndInSeconds() {
	return microTimeEnd * 1e-3;
    }
    

    public String toString() {
	return "(" + getBegin() + ", " + getEnd() + ")";
    }

}
