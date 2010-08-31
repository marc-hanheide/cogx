/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.util;

import elm.event.Event;

public class EventBuffer extends CircularBuffer<Event> {

    public EventBuffer(int capacity) throws ArrayIndexOutOfBoundsException {
	super(capacity);
    }
}


