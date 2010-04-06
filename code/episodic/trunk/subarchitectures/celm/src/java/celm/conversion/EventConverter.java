package celm.conversion;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Vector;

import celm.autogen.CELMCoreEvent;
import celm.autogen.CELMCueMatchMode;
import celm.autogen.CELMEventCue;
import celm.autogen.CELMEventLocation;
import celm.autogen.CELMEventSpecificFeaturesEntry;
import celm.autogen.CELMEventToStore;
import celm.autogen.CELMPartialEventToStore;
import celm.autogen.CELMStoredEvent;
import elm.event.Event;
import elm.event.EventID;
import elm.event.EventLocation;
import elm.event.EventLocationFactory;
import elm.event.EventSpecificFeatures;
import elm.event.EventTemplate;
import elm.event.EventType;
// import elm.event.PhysicalEntityID;
import elm.event.WKTParseException;

/**
 * EventConverter is a class which can perform all sorts of conversions between
 * the (core) ELM system's data structures and those of the C-ELM layer.
 * 
 * @author Dennis Stachowicz
 */
public class EventConverter {

	private EventLocationFactory elFactory = new EventLocationFactory();

	public CELMCoreEvent getCoreEvent(Event e) {

		byte[] data = e.getEventSpecificBinaryDataAsByteArray();

		return new CELMCoreEvent(e.getEventType().getName(), EventTimeConverter
				.toCEventTime(e.getTime()), new CELMEventLocation(e
				.getLocation().getWKTString()), e.isApexEvent(), e.getDegree(),
				eventIDVectorToLongArray(e.getSubEventIDs()),
				// physicalEntityIDVectorToStringArray(e.getPhysicalEntityIDs()),
				toCEventSpecificFeatures(e.getEventSpecificFeatures()),
				data == null ? new byte[0] : data);

	}

	public CELMStoredEvent getStoredEvent(Event e) {
		return new CELMStoredEvent(e.getEventID().getLong(), getCoreEvent(e));
	}

	public CELMEventToStore getEventToStore(Event e) {
		return new CELMEventToStore(getCoreEvent(e));
	}

	public CELMEventToStore getEventToStore(CELMPartialEventToStore e,
			CELMEventLocation location) {

		return new CELMEventToStore(getCoreEvent(e, location));
	}

	public CELMCoreEvent getCoreEvent(CELMPartialEventToStore e,
			CELMEventLocation location) {

		return new CELMCoreEvent(e.eventType, e.eventTime, location, true,
					 Event.degreeAtomic, new long[0], 
					 // e.physicalEntities,
					 e.eventSpecificFeatures, e.eventSpecificBinaryData);
	}

	public Event getEvent(CELMCoreEvent cevent) throws WKTParseException {

		EventType eventType = new EventType(cevent.eventType);
		EventLocation location = elFactory
				.fromString(cevent.location.wktString);

		return new ConvEvent(null, cevent.apexEvent, cevent.eventDegree,
				eventType, EventTimeConverter.toEventTime(cevent.eventTime),
				location, cevent.eventSpecificBinaryData,
				longArrayToEventIDVector(cevent.subevents),
				// stringArrayToPhysicalEntityIDVector(cevent.physicalEntities),
				toEventSpecificFeatures(cevent.eventSpecificFeatures));
	}

	public Event getEvent(CELMEventToStore cevent) throws WKTParseException {

		return getEvent(cevent.event);
	}

	public Event getEvent(CELMStoredEvent cevent) throws WKTParseException {

		EventType eventType = new EventType(cevent.event.eventType);
		EventLocation location = elFactory
				.fromString(cevent.event.location.wktString);

		return new ConvEvent(
				new EventID(cevent.eventID),
				cevent.event.apexEvent,
				cevent.event.eventDegree,
				eventType,
				EventTimeConverter.toEventTime(cevent.event.eventTime),
				location,
				cevent.event.eventSpecificBinaryData,
				longArrayToEventIDVector(cevent.event.subevents),
				// stringArrayToPhysicalEntityIDVector(cevent.event.physicalEntities),
				toEventSpecificFeatures(cevent.event.eventSpecificFeatures));
	}

	public CELMEventCue getEventCue(EventTemplate template)
			throws ConversionException {

		CELMEventCue cue = new CELMEventCue();

		cue.matchEventID = template.minEventID != null
				|| template.maxEventID != null;

		if (template.minEventID != null)
			cue.minEventID = template.minEventID.getLong();
		if (template.maxEventID != null)
			cue.maxEventID = template.maxEventID.getLong();

		if (template.eventType != null) {
			cue.matchEventType = true;
			cue.eventType = template.eventType.getName();
			cue.eventTypeIncludeSubtypes = !template.exactTypeMatch;
		} else {
			cue.matchEventType = false;
			cue.eventType = "";
		}
		cue.matchApexEvent = template.matchApexEvent;
		cue.apexEvent = template.apexEvent;

		cue.matchEventDegree = template.minDegree != elm.event.Event.degreeUndefined
				|| template.maxDegree != elm.event.Event.degreeUndefined;
		cue.minEventDegree = (template.minDegree != elm.event.Event.degreeUndefined ? template.minDegree
				: Integer.MIN_VALUE);
		cue.maxEventDegree = (template.maxDegree != elm.event.Event.degreeUndefined ? template.maxDegree
				: Integer.MAX_VALUE);

		cue.subeventMatchMode = getMatchMode(template.subEventMatchMode);
		cue.subevents = eventIDVectorToLongArray(template.subEventIDs);

		cue.supereventMatchMode = getMatchMode(template.superEventMatchMode);
		cue.superevents = eventIDVectorToLongArray(template.superEventIDs);

		// cue.physicalEntityMatchMode = getMatchMode(template.physicalEntityMatchMode);
		// cue.physicalEntities = physicalEntityIDVectorToStringArray(template.physicalEntityIDs);

		cue.timeMatchMode = getMatchMode(template.timeMatchMode);
		cue.eventTime = EventTimeConverter.toCEventTime(template.time);

		cue.locationMatchMode = getMatchMode(template.locationMatchMode);
		if (template.locationMatchMode != EventTemplate.noMatch)
			cue.location = new CELMEventLocation(template.location
					.getWKTString());
		else
			cue.location = new CELMEventLocation("");

		cue.esfMatchMode = getMatchMode(template.esfMatchMode);
		cue.esfRestrictMatchToKeys = template.esfRestrictMatchToKeys;
		cue.eventSpecificFeatures = toCEventSpecificFeatures(template.esf);

		if (template.binaryEventData == null) {
			cue.matchEventSpecificBinaryData = false;
			cue.eventSpecificBinaryData = new byte[0];
		} else {
			cue.matchEventSpecificBinaryData = true;
			cue.eventSpecificBinaryData = template.binaryEventData;
		}

		return cue;
	}

	public EventTemplate getEventTemplate(CELMEventCue cue)
			throws WKTParseException, ConversionException {

		EventTemplate template = new EventTemplate();

		if (cue.matchEventID) {
			template.minEventID = new EventID(cue.minEventID);
			template.maxEventID = new EventID(cue.maxEventID);
		}

		if (cue.matchEventType) {
			template.eventType = new EventType(cue.eventType);
			template.exactTypeMatch = !cue.eventTypeIncludeSubtypes;
		}

		template.matchApexEvent = cue.matchApexEvent;
		template.apexEvent = cue.apexEvent; // getTemplateTriBool(cue.apexEvent);

		if (cue.matchEventDegree) {
			template.minDegree = cue.minEventDegree;
			template.maxDegree = cue.maxEventDegree;
		}

		template.subEventIDs = longArrayToEventIDVector(cue.subevents);
		template.subEventMatchMode = getMatchMode(cue.subeventMatchMode);

		template.superEventIDs = longArrayToEventIDVector(cue.superevents);
		template.superEventMatchMode = getMatchMode(cue.supereventMatchMode);

		// template.physicalEntityIDs = stringArrayToPhysicalEntityIDVector(cue.physicalEntities);
		// template.physicalEntityMatchMode = getMatchMode(cue.supereventMatchMode);

		template.time = EventTimeConverter.toEventTime(cue.eventTime);
		template.timeMatchMode = getMatchMode(cue.timeMatchMode);

		if (cue.location != null) {
			template.location = elFactory.fromString(cue.location.wktString);
		}
		else {
			assert(cue.locationMatchMode == CELMCueMatchMode.noMatch);
		}
		template.locationMatchMode = getMatchMode(cue.locationMatchMode);

		template.esfMatchMode = getMatchMode(cue.esfMatchMode);
		template.esfRestrictMatchToKeys = cue.esfRestrictMatchToKeys;
		template.esf = toEventSpecificFeatures(cue.eventSpecificFeatures);

		if (cue.matchEventSpecificBinaryData)
			template.binaryEventData = cue.eventSpecificBinaryData;

		return template;
	}

	public static CELMCueMatchMode getMatchMode(int imm)
			throws ConversionException {

		if (imm == EventTemplate.noMatch)
			return CELMCueMatchMode.noMatch;
		else if (imm == EventTemplate.matchExact)
			return CELMCueMatchMode.matchExact;
		else if (imm == EventTemplate.matchSubset)
			return CELMCueMatchMode.matchSubset;
		else if (imm == EventTemplate.matchSuperset)
			return CELMCueMatchMode.matchSuperset;
		else if (imm == EventTemplate.matchIntersectionNotEmpty)
			return CELMCueMatchMode.matchIntersectionNotEmpty;
		else
			throw new ConversionException("unknown match mode: " + imm);

	}

	public static int getMatchMode(CELMCueMatchMode cmm)
			throws ConversionException {

		if (cmm.value() == CELMCueMatchMode.noMatch.value())
			return EventTemplate.noMatch;
		else if (cmm.value() == CELMCueMatchMode.matchExact.value())
			return EventTemplate.matchExact;
		else if (cmm.value() == CELMCueMatchMode.matchSubset.value())
			return EventTemplate.matchSubset;
		else if (cmm.value() == CELMCueMatchMode.matchSuperset.value())
			return EventTemplate.matchSuperset;
		else if (cmm.value() == CELMCueMatchMode.matchIntersectionNotEmpty
				.value())
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

	/*
	public static Vector<PhysicalEntityID> stringArrayToPhysicalEntityIDVector(
			String[] s) {

		if (s == null)
			return null;

		Vector<PhysicalEntityID> ov = new Vector<PhysicalEntityID>(s.length);

		for (String si : s)
			ov.add(new PhysicalEntityID(si));

		return ov;
	}
	*/

	public static long[] eventIDVectorToLongArray(Vector<EventID> ev) {

		if (ev == null)
			return new long[0];

		long[] l = new long[ev.size()];
		for (int i = 0; i < l.length; i++)
			l[i] = ev.get(i).getLong();

		return l;
	}

	/*
	public static String[] physicalEntityIDVectorToStringArray(
			Vector<PhysicalEntityID> ev) {

		if (ev == null)
			return new String[0];

		String[] s = new String[ev.size()];
		for (int i = 0; i < s.length; i++)
			s[i] = ev.get(i).getString();

		return s;
	}
	*/

	public static CELMEventSpecificFeaturesEntry[] toCEventSpecificFeatures(
			EventSpecificFeatures data) {

		if (data == null)
			return new CELMEventSpecificFeaturesEntry[0];

		CELMEventSpecificFeaturesEntry[] entryArray = new CELMEventSpecificFeaturesEntry[data
				.size()];

		Iterator<String> keyIterator = data.keySet().iterator();

		for (int i = 0; i < entryArray.length && keyIterator.hasNext(); i++) {
			String featureName = keyIterator.next();
			Iterator<String> valuesIterator = data.get(featureName).iterator();

			Vector<String> valuesVector = new Vector<String>();
			while (valuesIterator.hasNext())
				valuesVector.add(valuesIterator.next());

			entryArray[i] = new CELMEventSpecificFeaturesEntry(featureName,
					valuesVector.toArray(new String[0]));
		}

		return entryArray;
	}

	public static EventSpecificFeatures toEventSpecificFeatures(
			CELMEventSpecificFeaturesEntry[] entryArray) {

		if (entryArray == null)
			return null;

		EventSpecificFeatures data = new EventSpecificFeatures(
				entryArray.length);

		for (int i = 0; i < entryArray.length; i++) {
			HashSet<String> hs = new HashSet<String>(
					entryArray[i].featureValues.length);
			for (int j = 0; j < entryArray[i].featureValues.length; j++)
				hs.add(entryArray[i].featureValues[j]);
			data.put(entryArray[i].featureName, hs);
		}

		return data;
	}

}
