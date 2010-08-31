/** _
 *  part of the CELM system.
 *  @author Dennis Stachowicz
 */

package celm.conversion;

import elm.event.*;

import java.util.Vector;

/**
 * Conversion event. 
 * 
 * 
 * @author nah
 *
 */
class ConvEvent extends Event {

    protected ConvEvent(EventID eventID,
			boolean apex,
			int degree,
			EventType eventType, 
			EventTime time, 
			EventLocation location,
			byte[] data,
			Vector<EventID> subEventIDs,
			// Vector<PhysicalEntityID> physicalEntityIDs,
			EventSpecificFeatures eventSpecificFeatures) {
	super(eventID, 
	      apex, 
	      degree, 
	      eventType, 
	      time, 
	      location, 
	      data, 
	      subEventIDs, 
	      null, // physicalEntityIDs, 
	      eventSpecificFeatures);
	
    }

}
