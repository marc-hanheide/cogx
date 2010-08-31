/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import java.util.Vector;
import java.io.IOException;

public class AtomicEvent extends Event {

    /**
     *  Constructor for atomic events, i.e. events which have no sub-events.
     */
    public AtomicEvent(EventType eventType, 
		       EventTime time, 
		       EventLocation location,
		       EventSpecificBinaryData data,
		       Vector<PhysicalEntityID> physicalEntityIDs,
		       EventSpecificFeatures esf) 

	throws IOException {

	super(null, true, degreeAtomic, eventType, time, location, data, null, physicalEntityIDs, esf);
    }

    /**
     *  Constructor for atomic events, i.e. events which have no sub-events.
     */
    public AtomicEvent(EventType eventType, 
		       EventTime time, 
		       EventLocation location,
		       byte[] data,
		       Vector<PhysicalEntityID> physicalEntityIDs,
		       EventSpecificFeatures esf) {
	super(null, true, degreeAtomic, eventType, time, location, data, null, physicalEntityIDs, esf);
    }

}
