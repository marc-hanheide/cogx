/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import java.io.IOException;
import java.io.Serializable;
import java.util.Vector;


public class Event {

    public static final int degreeAtomic    =  0;
    public static final int degreeUndefined =  degreeAtomic - 1;

    private EventID eventID        = null;
    private boolean apex           = true;
    private int degree             = degreeUndefined;
    private EventType eventType    = null;
    private EventTime time         = null;
    private EventLocation location = null;
    
    private Vector<EventID>                 subEventIDs             = null;
    private Vector<PhysicalEntityID>        physicalEntityIDs       = null;
    private EventSpecificFeatures           eventSpecificFeatures   = null;
    private byte[]                          binaryEventData         = null;
    
    /**
     *  Constructor for events already registered with the database 
     *  and hence having an event ID or for other internal use.
     */
    protected Event(EventID eventID,
		    boolean apex,
		    int degree,
		    EventType eventType, 
		    EventTime time, 
		    EventLocation location,
		    byte[] data,
		    Vector<EventID> subEventIDs,
		    Vector<PhysicalEntityID> physicalEntityIDs,
		    EventSpecificFeatures eventSpecificFeatures) {

	init(apex, degree, eventType, time, location, 
	     data, subEventIDs, physicalEntityIDs, eventSpecificFeatures);
	this.eventID = eventID;
    }

    /**
     *  Constructor for events already registered with the database 
     *  and hence having an event ID or for other internal use.
     */
    protected Event(EventID eventID,
		    boolean apex,
		    int degree,
		    EventType eventType, 
		    EventTime time, 
		    EventLocation location,
		    EventSpecificBinaryData data,
		    Vector<EventID> subEventIDs,
		    Vector<PhysicalEntityID> physicalEntityIDs,
		    EventSpecificFeatures eventSpecificFeatures)

	throws IOException {


	init(apex, degree, eventType, time, location, 
	     EventSpecificBinaryDataIO.toByteArray(data), subEventIDs, physicalEntityIDs, eventSpecificFeatures);
	this.eventID = eventID;
    }

    protected void init(boolean apex,
			int degree,
			EventType eventType, 
			EventTime time, 
			EventLocation location,
		        byte[] data,
		        Vector<EventID> subEventIDs,
		        Vector<PhysicalEntityID> physicalEntityIDs,
			EventSpecificFeatures eventSpecificFeatures) {
	
	this.apex = apex;
	this.degree = degree;
	this.eventType = eventType;
	this.time = time;
	this.location = location;
	this.binaryEventData = data;
	this.subEventIDs = subEventIDs;
	this.physicalEntityIDs = physicalEntityIDs;
	this.eventSpecificFeatures = eventSpecificFeatures;
    }


    public String toString() {
	return eventToString(false);
    }
    
    public String eventToString(boolean tryConvertESBData) {
	
	String esbdString = ""; 
	
	if (tryConvertESBData) {
	    try {
		EventSpecificBinaryData esbd = getEventSpecificBinaryDataAsESBDObject();
		esbdString = "\"" + esbd.toString() + "\"";
	    }
	    catch (Exception e) {
		esbdString = "Conversion failed. Maybe this is not a (correctly) serialized object?";
	    }
	}
	
	String s = "event {" +
	    "\n   id:                                 " + (hasEventIDAssigned() ? eventID : "none") +
	    "\n   type:                               " + getEventType() +
	    "\n   is atomic:                          " + isAtomic() +
	    "\n   is apex event:                      " + isApexEvent() +
	    "\n   degree:                             " + getDegree() +
	    "\n   time:                               " + getTime() + 
	    "\n   location:                           " + getLocation() +
	    // temporarily switch on if needed for debugging:
	    // "\n   event data:         " + getEventSpecificBinaryDataAsObject() +
	    "\n   subevent IDs:                       " + subEventIDs +
	    "\n   physical entity IDs:                " + physicalEntityIDs +
	    "\n   event specific features:            " + eventSpecificFeatures + 
	    "\n   has event specific binary data:     " + hasEventSpecificBinaryData() +
	    (tryConvertESBData 
		? "\n   toString(event spec. binary data):  " + esbdString : "") +
	    "\n}";

	return s;
    }


    public boolean isAtomic() {
	// return (subEventIDs == null || subEventIDs.size() == 0 ? true : false);
	return (degree == degreeAtomic);
    }
    
    // eqivalent to hasSubEvents() as long as there is no forgetting / interference / etc.
    public boolean isComplex() {
	// return !isAtomic();
	return (degree > degreeAtomic);
    }

    public boolean hasEventIDAssigned() {
	return (eventID != null);
    }

    public EventID getEventID() 
        // throws EventIDException
    {
	// if (!hasEventIDAssigned())
	//    throw new EventIDException("no ID assigned!");
	return eventID;
    }

    public void assignEventID(EventID id) throws EventIDException {
	if (hasEventIDAssigned())
	    throw new EventIDException("this Event object already has an ID assigned!");
	eventID = id;
    }

    public boolean isApexEvent() {
	return apex;
    }

    public void setApexEvent(boolean value) {
	apex = value;
    }

    public int getDegree() {
	return degree;
    }

    public void setDegree(int degree) {
	this.degree = degree;
    }

    public EventTime getTime() {
	return time;
    }

    public void setTime(EventTime t) {
	time = t;
    }
    
    public EventType getEventType() {
	return eventType;
    }

    public void setEventType(EventType t) {
	eventType = t;
    }
    
    public EventLocation getLocation() {
	return location;
    }

    public void setLocation(EventLocation l) {
	location = l;
    }

    public Vector<PhysicalEntityID> getPhysicalEntityIDs() {
	return physicalEntityIDs;
    }

    public void setPhysicalEntityIDs(Vector<PhysicalEntityID> newPhysicalEntityIDs) {
	physicalEntityIDs = newPhysicalEntityIDs;
    }

    public boolean hasEventSpecificBinaryData() {
	return (binaryEventData != null);
    }

    public byte[] getEventSpecificBinaryDataAsByteArray() {
	return binaryEventData;
    }

    public void setEventSpecificBinaryData(byte[] data) {
	binaryEventData = data;
    }
    public void setEventSpecificBinaryData(EventSpecificBinaryData data) throws java.io.IOException {
	binaryEventData = EventSpecificBinaryDataIO.toByteArray(data);
    }

    public Object getEventSpecificBinaryDataAsRawObject() 
	throws IOException, ClassNotFoundException, ClassCastException {
	return EventSpecificBinaryDataIO.objectFromByteArray(binaryEventData);
    }

    public EventSpecificBinaryData getEventSpecificBinaryDataAsESBDObject() 
	throws IOException, ClassNotFoundException, ClassCastException {
	return EventSpecificBinaryDataIO.fromByteArray(binaryEventData);
    }

    public boolean hasEventSpecificFeatures() {
	return (eventSpecificFeatures != null && eventSpecificFeatures.totalNumberOfPairs() > 0);
    }
    
    public EventSpecificFeatures getEventSpecificFeatures() {
	return eventSpecificFeatures;
    }

    // eqivalent to isComplex() as long as there is no forgetting / interference / etc.
    public boolean hasSubEvents() {
	return (subEventIDs != null && subEventIDs.size() > 0);
    }
    
    public Vector<EventID> getSubEventIDs() {
	return subEventIDs;
    }
    
    public void setSubEventIDs(Vector<EventID> subEventIDs) {
	this.subEventIDs = subEventIDs;
    }

}

