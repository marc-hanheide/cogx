/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import elm.event.*;
import elm.dbio.ELMDatabaseWriter;

import          java.sql.Connection;
import          java.sql.SQLException;
import          java.sql.PreparedStatement;
import		java.sql.ResultSet;
import		java.sql.Statement;

import          java.io.Serializable;
import          java.io.IOException;
import		java.util.Date;
import          java.util.Vector;


// only meant as a wrapper to keep old prototype code running
// not an actual part of the ELM library!
// use ELMDatabaseWriter!
class EventInserter extends ELMDatabaseWriter {



	public EventInserter(Connection c) throws SQLException {
	    super(c);
	}



 
	// returns the event_id assigned
        // currently works only for atomic events!!!
	public Event storeEvent(// eventBinary not implemented yet,
			String eventTypeString,
			long microTimeBegin,
			long microTimeEnd,
			// loc can be computed from subevents if this is a
			// composite event, pass as null then
			EventLocation loc,
			EventSpecificBinaryData binaryEventData,
			Vector<Event> subEvents,   // ignored!!!
			Vector<PhysicalEntityID> entitiesInvolved) 
	// currently no associated data
	    throws SQLException, IOException, EventIDException 
        {
	    EventType etype = new EventType(eventTypeString);
	    EventTime time = new EventTime(microTimeBegin, microTimeEnd);
	    Event event = null;

	    // if (subEvents == null)
	    // PhysicalEntityID[] oids = new PhysicalEntityID[entitiesInvolved.length];
// 	    for (int i = 0; i < oids.length; i++)
// 		oids[i] = new PhysicalEntityID(entitiesInvolved[i]);


	    // event = new AtomicEvent(etype, time, loc, binaryEventData, entitiesInvolved);

	    /*
	    else {
		// ignoring time, location, data and objects for now...
		// could also be set...()
		
		Vector<long> ev = new Vector<long>();
	        ev.copyInto(subEvents);
		event = new ComplexEvent(etype, ev);
	    }
	    */

	    
	    if (subEvents == null || subEvents.size() == 0)
		event = new AtomicEvent(etype, time, loc, binaryEventData, entitiesInvolved, null);
	    else
		event = new ComplexEvent(etype, subEvents);
	    
	    storeEvent(event);
	    return event;
	}


}
