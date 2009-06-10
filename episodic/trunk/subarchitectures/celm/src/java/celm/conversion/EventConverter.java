package celm.conversion;

import elm.event.*;
import celm.autogen.*;

import java.util.Vector;
import java.util.Set;
import java.util.HashSet;
import java.util.Iterator;

/**
 *  EventConverter is a class which can perform all sorts of 
 *  conversions between the (core) ELM system's data structures
 *  and those of the C-ELM layer.
 *
 *  @author Dennis Stachowicz
 */
public class EventConverter {

    private EventLocationFactory elFactory = new EventLocationFactory();


    public CELM_CoreEvent getCoreEvent(Event e) {

	byte[] data = e.getEventSpecificBinaryDataAsByteArray();

	return new CELM_CoreEvent(e.getEventType().getName(),
				  EventTimeConverter.toCEventTime(e.getTime()),
				  new CELM_EventLocation(e.getLocation().getWKTString()),
				  e.isApexEvent(),
				  e.getDegree(),
				  eventIDVectorToLongArray(e.getSubEventIDs()),
				  physicalEntityIDVectorToStringArray(e.getPhysicalEntityIDs()),
				  toCEventSpecificFeatures(e.getEventSpecificFeatures()),
				  data == null ? new byte[0] : data);
				  
    }

    public CELM_StoredEvent getStoredEvent(Event e) {

	return new CELM_StoredEvent(e.getEventID().getLong(),
				    getCoreEvent(e));
    }

    public CELM_EventToStore getEventToStore(Event e) {

	return new CELM_EventToStore(getCoreEvent(e));
    }
    

    public CELM_EventToStore getEventToStore(CELM_PartialEventToStore e, 
					     CELM_EventLocation location) {

	return new CELM_EventToStore(getCoreEvent(e, location));
    }


    public CELM_CoreEvent getCoreEvent(CELM_PartialEventToStore e, 
				       CELM_EventLocation location) {

	return new CELM_CoreEvent(e.m_eventType,
				  e.m_eventTime,
				  location,
				  true,
				  Event.degreeAtomic,
				  new long[0],
				  e.m_physicalEntities,
				  e.m_eventSpecificFeatures,
				  e.m_eventSpecificBinaryData);	
    }




    public Event getEvent(CELM_CoreEvent cevent) throws WKTParseException {
		
	EventType eventType = new EventType(cevent.m_eventType);
	EventLocation location = elFactory.fromString(cevent.m_location.m_wktString);
	
	return new ConvEvent(null,
			     cevent.m_apexEvent,
			     cevent.m_eventDegree,
			     eventType,
			     EventTimeConverter.toEventTime(cevent.m_eventTime),
			     location,
			     cevent.m_eventSpecificBinaryData,
			     longArrayToEventIDVector(cevent.m_subevents),
			     stringArrayToPhysicalEntityIDVector(cevent.m_physicalEntities),
			     toEventSpecificFeatures(cevent.m_eventSpecificFeatures));
    }



    public Event getEvent(CELM_EventToStore cevent) throws WKTParseException {
    
	return getEvent(cevent.m_event);
    }

    public Event getEvent(CELM_StoredEvent cevent) throws WKTParseException {

	EventType eventType = new EventType(cevent.m_event.m_eventType);
	EventLocation location = elFactory.fromString(cevent.m_event.m_location.m_wktString);

	return new ConvEvent(new EventID(cevent.m_eventID),
			     cevent.m_event.m_apexEvent,
			     cevent.m_event.m_eventDegree,
			     eventType,
			     EventTimeConverter.toEventTime(cevent.m_event.m_eventTime),
			     location,
			     cevent.m_event.m_eventSpecificBinaryData,
			     longArrayToEventIDVector(cevent.m_event.m_subevents),
			     stringArrayToPhysicalEntityIDVector(cevent.m_event.m_physicalEntities),
			     toEventSpecificFeatures(cevent.m_event.m_eventSpecificFeatures));
    }


    public CELM_EventCue getEventCue(EventTemplate template) 
	throws ConversionException {

	CELM_EventCue cue = new CELM_EventCue();

	cue.m_matchEventID = template.minEventID != null 
	                  || template.maxEventID != null;
	
	if (template.minEventID != null)
	    cue.m_minEventID = template.minEventID.getLong();
	if (template.maxEventID != null)
	    cue.m_maxEventID = template.maxEventID.getLong();

	if (template.eventType != null) {
	    cue.m_matchEventType = true;
	    cue.m_eventType = template.eventType.getName();
	    cue.m_eventTypeIncludeSubtypes = !template.exactTypeMatch;
	}
	else {
	    cue.m_matchEventType = false;
	    cue.m_eventType = "";
	}
	cue.m_matchApexEvent = template.matchApexEvent;
	cue.m_apexEvent = template.apexEvent;

	   
	cue.m_matchEventDegree = 
	       template.minDegree != elm.event.Event.degreeUndefined 
	    || template.maxDegree != elm.event.Event.degreeUndefined;
	cue.m_minEventDegree = 
	    (template.minDegree != elm.event.Event.degreeUndefined ?
	     template.minDegree : Integer.MIN_VALUE);
	cue.m_maxEventDegree = 
	    (template.maxDegree != elm.event.Event.degreeUndefined ?
	     template.maxDegree : Integer.MAX_VALUE);
	    
	cue.m_subeventMatchMode = getMatchMode(template.subEventMatchMode);
	cue.m_subevents = eventIDVectorToLongArray(template.subEventIDs);

	cue.m_supereventMatchMode = getMatchMode(template.superEventMatchMode);
	cue.m_superevents = eventIDVectorToLongArray(template.superEventIDs);

	cue.m_physicalEntityMatchMode = getMatchMode(template.physicalEntityMatchMode);
	cue.m_physicalEntities = physicalEntityIDVectorToStringArray(template.physicalEntityIDs);

	cue.m_timeMatchMode = getMatchMode(template.timeMatchMode);
	cue.m_eventTime = EventTimeConverter.toCEventTime(template.time);

	cue.m_locationMatchMode = getMatchMode(template.locationMatchMode);
	if (template.locationMatchMode != EventTemplate.noMatch)
	    cue.m_location = new CELM_EventLocation(template.location.getWKTString());
	else
	    cue.m_location = new CELM_EventLocation("");

	cue.m_esfMatchMode = getMatchMode(template.esfMatchMode);
	cue.m_esfRestrictMatchToKeys = template.esfRestrictMatchToKeys;
	cue.m_eventSpecificFeatures = toCEventSpecificFeatures(template.esf);
	
	if (template.binaryEventData == null) {
	    cue.m_matchEventSpecificBinaryData = false;
	    cue.m_eventSpecificBinaryData = new byte[0];
	}
	else {
	    cue.m_matchEventSpecificBinaryData = true;
	    cue.m_eventSpecificBinaryData = template.binaryEventData;
	}

	return cue;
    }


    public EventTemplate getEventTemplate(CELM_EventCue cue)
	throws WKTParseException, ConversionException {
	
	EventTemplate template = new EventTemplate();
	
	if (cue.m_matchEventID) {
	    template.minEventID          = new EventID(cue.m_minEventID);
	    template.maxEventID          = new EventID(cue.m_maxEventID);
	}

	if (cue.m_matchEventType) {
	    template.eventType           = new EventType(cue.m_eventType);
	    template.exactTypeMatch      = !cue.m_eventTypeIncludeSubtypes;
	}

	template.matchApexEvent          = cue.m_matchApexEvent;
	template.apexEvent               = cue.m_apexEvent; // getTemplateTriBool(cue.m_apexEvent);

	if (cue.m_matchEventDegree) {
	    template.minDegree           = cue.m_minEventDegree;
	    template.maxDegree           = cue.m_maxEventDegree;
	}
	
	template.subEventIDs             = longArrayToEventIDVector(cue.m_subevents);
	template.subEventMatchMode       = getMatchMode(cue.m_subeventMatchMode);

	template.superEventIDs           = longArrayToEventIDVector(cue.m_superevents);
	template.superEventMatchMode     = getMatchMode(cue.m_supereventMatchMode);

	template.physicalEntityIDs       = stringArrayToPhysicalEntityIDVector(cue.m_physicalEntities);
	template.physicalEntityMatchMode = getMatchMode(cue.m_supereventMatchMode);

	template.time                    = EventTimeConverter.toEventTime(cue.m_eventTime);
	template.timeMatchMode           = getMatchMode(cue.m_timeMatchMode);

	template.location                = elFactory.fromString(cue.m_location.m_wktString);
	template.locationMatchMode       = getMatchMode(cue.m_locationMatchMode);

	template.esfMatchMode            = getMatchMode(cue.m_esfMatchMode);
	template.esfRestrictMatchToKeys  = cue.m_esfRestrictMatchToKeys;
	template.esf                     = toEventSpecificFeatures(cue.m_eventSpecificFeatures);
	
	if (cue.m_matchEventSpecificBinaryData)
	    template.binaryEventData     = cue.m_eventSpecificBinaryData;
	
	return template;
    }

    public static CELM_CueMatchMode getMatchMode(int imm)
	throws ConversionException {
	
	if (imm == EventTemplate.noMatch)
	    return CELM_CueMatchMode.noMatch;
	else if (imm == EventTemplate.matchExact)
	    return CELM_CueMatchMode.matchExact;
	else if (imm == EventTemplate.matchSubset)
	    return CELM_CueMatchMode.matchSubset;
	else if (imm == EventTemplate.matchSuperset)
	    return CELM_CueMatchMode.matchSuperset;
	else if (imm == EventTemplate.matchIntersectionNotEmpty)
	    return CELM_CueMatchMode.matchIntersectionNotEmpty;
	else
	    throw new ConversionException("unknown match mode: " + imm);

    }

    public static int getMatchMode(CELM_CueMatchMode cmm)
	throws ConversionException {
	
	if (cmm.value() == CELM_CueMatchMode.noMatch.value())
	    return EventTemplate.noMatch;
	else if (cmm.value() == CELM_CueMatchMode.matchExact.value())
	    return EventTemplate.matchExact;
	else if (cmm.value() == CELM_CueMatchMode.matchSubset.value())
	    return EventTemplate.matchSubset;
	else if (cmm.value() == CELM_CueMatchMode.matchSuperset.value())
	    return EventTemplate.matchSuperset;
	else if (cmm.value() == CELM_CueMatchMode.matchIntersectionNotEmpty.value())
	    return EventTemplate.matchIntersectionNotEmpty;
	else
	    throw new ConversionException("unknown match mode: " + cmm.value());
    }

    public static Vector<EventID> longArrayToEventIDVector(long[] l) {

	if (l == null)
	    return null;

	Vector<EventID> ev = new Vector<EventID>(l.length);

	for (long li : l)
	    ev.add(new EventID(li));

	return ev;
    }

    public static Vector<PhysicalEntityID> stringArrayToPhysicalEntityIDVector(String[] s) {

	if (s == null)
	    return null;

	Vector<PhysicalEntityID> ov = new Vector<PhysicalEntityID>(s.length);

	for (String si : s)
	    ov.add(new PhysicalEntityID(si));

	return ov;
    }

    public static long[] eventIDVectorToLongArray(Vector<EventID> ev) {

	if (ev == null)
	    return new long[0];
	
	long[] l = new long[ev.size()];
	for (int i = 0; i < l.length; i++)
	    l[i] = ev.get(i).getLong();

	return l;
    }


    public static String[] physicalEntityIDVectorToStringArray(Vector<PhysicalEntityID> ev) {
	
	if (ev == null)
	    return new String[0];

	String[] s = new String[ev.size()];
	for (int i = 0; i < s.length; i++)
	    s[i] = ev.get(i).getString();

	return s;
    }

    public static CELM_EventSpecificFeaturesEntry[] toCEventSpecificFeatures(EventSpecificFeatures data) {

	if (data == null)
	    return new CELM_EventSpecificFeaturesEntry[0];

	CELM_EventSpecificFeaturesEntry[] entryArray = new CELM_EventSpecificFeaturesEntry[data.size()];

	Iterator<String> keyIterator = data.keySet().iterator();

	for (int i = 0; i < entryArray.length && keyIterator.hasNext(); i++) {
	    String featureName = keyIterator.next();
	    Iterator<String> valuesIterator = data.get(featureName).iterator();
		    
	    Vector<String> valuesVector = new Vector<String>();
	    while (valuesIterator.hasNext()) 
		valuesVector.add(valuesIterator.next());
		    
	    entryArray[i] = 
		new CELM_EventSpecificFeaturesEntry(featureName, valuesVector.toArray(new String[0]));
	}
	
	return entryArray;
    }

    public static EventSpecificFeatures toEventSpecificFeatures(CELM_EventSpecificFeaturesEntry[] entryArray) {

	if (entryArray == null)
	    return null;

	EventSpecificFeatures data = new EventSpecificFeatures(entryArray.length);

	for (int i = 0; i < entryArray.length; i++) {
	    HashSet<String> hs = new HashSet<String>(entryArray[i].m_featureValues.length);
	    for (int j = 0; j < entryArray[i].m_featureValues.length; j++) 
		hs.add(entryArray[i].m_featureValues[j]);
	    data.put(entryArray[i].m_featureName, hs);
	}

	return data;
    }
    
}


