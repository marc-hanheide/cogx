/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.dbio;

import elm.event.*;

import java.util.Vector;

class DBEvent extends Event {

    protected DBEvent(EventID eventID,
		      boolean apex,
		      int degree,
		      EventType eventType, 
		      EventTime time, 
		      EventLocation location,
		      byte[] data,
		      Vector<EventID> subEventIDs,
		      Vector<PhysicalEntityID> physicalEntityIDs,
		      EventSpecificFeatures esf) {
	super(eventID, apex, degree, eventType, time, location, data, subEventIDs, physicalEntityIDs, esf);
	
    }

}
