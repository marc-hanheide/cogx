/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import java.util.Random;
// import java.util.Date;
import java.util.Vector;
import java.util.Iterator;
// import java.util.HashSet;
// 
// import java.io.IOException;

import elm.event.*;
import elm.util.EventBuffer;

public class BMQuerySimulator {
    
    public static final boolean verbose            = false;
    
    public static final int defaultEventBufferSize = 25000;
    public static final int minNoEventsInBuffer    = 100; // save all events of the first few rounds
    public static final int defMinQueryComponents  = 2;
    public static final int defMaxQueryComponents  = 2;
 
    public static final double ptBuffer        = 20;
    
    public static final double pID             = 0.08;
    public static final double pType           = 0.08 + pID;
    public static final double pTime           = 0.16 + pType;
    public static final double pLocation       = 0.16 + pTime;
    public static final double pESF            = 0.32 + pLocation;
    public static final double pESBD           = 0.04 + pESF;
    public static final double pSubevent       = 0.16 + pESBD;
    public static final double pMax            = pSubevent;     // for reporting and checking

/*    public static final double pID             = 0.0;
    public static final double pType           = 0.0 + pID;
    public static final double pTime           = 0.0 + pType;
    public static final double pLocation       = 0.0 + pTime;
    public static final double pESF            = 0.7 + pLocation;
    public static final double pSubevent       = 0.3 + pESF;*/
    
    
    public static final double pIDSingle       = 0.5;
    public static final double pTypeExact      = 0.5;
    public static final double pESFMatchToKeys = 0.5;
    
    public static final int    rangeMargin     = 20;
    public static final int    maxMatchMode    = 3;
    
    public static final int    numberOfQueryTypes = 25;
    
/*    protected Event[] eventBuffer = new Event[eventBufferSize];
    protected int     bufPtr      = 0;
    protected int     noEvents    = 0;*/
    protected EventBuffer eventBuffer  = new EventBuffer(defaultEventBufferSize);
    protected int     bufPtr           = 0;
    protected double  pBufferInclusion = 0.25; 
    
    protected EventLocationFactory elFactory = null;
    protected Random rng = new Random();
    
        
    public BMQuerySimulator(EventLocationFactory elFactory) {
	this.elFactory = elFactory;
	if (verbose)
	    System.out.println("BMQuerySim: pMax = " + pMax);
    }
    
    public BMQuerySimulator(EventLocationFactory elFactory, 
			    int eventBufferSize, 
			    double pIncludeInBuffer) {
				
	this.elFactory = elFactory;
	this.eventBuffer = new EventBuffer(eventBufferSize);
	this.pBufferInclusion = pIncludeInBuffer;
	if (verbose)
	    System.out.println("BMQuerySim: pMax = " + pMax);
    }
    
    
//     synchronized public void newStoredEvent(Event e) {
// 	
// 	int pos = rng.nextInt(eventBufferSize);
// 
// 	if (eventBuffer[pos] == null)
// 	    noEvents++;
// 	eventBuffer[pos] = e;
//     }
    
    synchronized public void newStoredEvent(Event e) {
	
	if (eventsInBuffer() < minNoEventsInBuffer || rng.nextDouble() < pBufferInclusion)
	    eventBuffer.addOrOverwrite(e);
    }    
    
/*    synchronized public int eventsInBuffer() {
	return noEvents;
    }
 */
    synchronized public int eventsInBuffer() {
	return eventBuffer.size();
    }
    
/*    synchronized protected Event getNextEvent() throws RuntimeException {
	
	if (eventsInBuffer() < 1)
	    throw new RuntimeException("no events in buffer!");
	do {
	    bufPtr++;
	    if (bufPtr >= eventBufferSize)
		bufPtr = 0;
	} while (eventBuffer[bufPtr] == null);
	return eventBuffer[bufPtr];
    }*/
    
    synchronized protected Event getNextEvent() throws RuntimeException {
	
	if (eventsInBuffer() < 1)
	    throw new RuntimeException("no events in buffer!");

	bufPtr = rng.nextInt(eventBuffer.size());
	return eventBuffer.get(bufPtr);
    }
    
    
//     synchronized protected Event getCurrentEvent() {
// 	return eventBuffer[bufPtr];
//     }
    
    synchronized protected Event getCurrentEvent() {
	return eventBuffer.get(bufPtr);
    }
    
    synchronized public void flushBuffer() throws elm.util.CircularBufferException {
	eventBuffer.flush();
	bufPtr = 0;
    }


    protected void idQ(EventTemplate t) {
	
	double v = rng.nextDouble();
	
	if (v < pIDSingle) 
	    idQ(t, true);
	else 
	    idQ(t, false);
    }
    
    protected void idQ(EventTemplate t, boolean singleQuery) {
	
	if (singleQuery) {
	    t.minEventID = t.maxEventID = getCurrentEvent().getEventID();
	}
	else {
	    t.minEventID = new EventID(getCurrentEvent().getEventID().getLong() - (rangeMargin / 2));
	    t.maxEventID = new EventID(getCurrentEvent().getEventID().getLong() + (rangeMargin / 2));
	}	
    }    
    
    protected EventType getASuperType(EventType t) {
	
	 // ??????
	 // throw new RuntimeException("not implemented yet");
	String currentName = t.getName();
	return new EventType(currentName.substring(0, currentName.length() - 2));
    }
    
    protected void typeQ(EventTemplate t) {
	
	double v = rng.nextDouble();
	
	if (v < pTypeExact) 
	    typeQ(t, true);
	else 
	    typeQ(t, false);
    }    
    
    protected void typeQ(EventTemplate t, boolean exact) {
	
	if (exact) {
	    t.eventType = getCurrentEvent().getEventType();
	    t.exactTypeMatch = true;
	}
	else { 
	    t.eventType = getASuperType(getCurrentEvent().getEventType());
	    t.exactTypeMatch = false;
	}	
    }    

    
    protected int getMatchMode() {
	return rng.nextInt(maxMatchMode + 1);
    }
    
    protected void timeQ(EventTemplate t) {
	
	timeQ(t, getMatchMode());
    }    
    
    protected void timeQ(EventTemplate t, int matchMode) {
	
	t.timeMatchMode = matchMode;
	
	// copy and add/subtract margin?
	t.time = getCurrentEvent().getTime();
    }    

    protected void locationQ(EventTemplate t) {
	
	locationQ(t, getMatchMode());
    }    
    
    protected void locationQ(EventTemplate t, int matchMode) {
	
	t.locationMatchMode = matchMode;
	
	// copy and add/subtract margin?
	t.location = getCurrentEvent().getLocation();
    }    

    protected void esfQ(EventTemplate t) {
	
	double v = rng.nextDouble();
	
	if (v < pESFMatchToKeys) 
	    esfQ(t, getMatchMode(), true);
	else
	    esfQ(t, getMatchMode(), false);

    }    
    
    
    protected void esfQ(EventTemplate t, int matchMode, boolean keysOnly) {
	
	double v = rng.nextDouble();
	
	t.esfRestrictMatchToKeys = keysOnly;
	t.esfMatchMode = matchMode;
	
	if (verbose)
	    System.out.print("...matchMode: " + t.esfMatchMode + "...");
	
	// copy and add/subtract margin?
			
	/*
	if (t.esfRestrictMatchToKeys) {
	    Vector<String> emptyVector = new Vector<String>();
	    t.esf = new EventSpecificFeatures();
	    Iterator<String> keyIterator = getCurrentEvent().getEventSpecificFeatures().keySet().iterator();
    
	    while (keyIterator.hasNext()) 
		t.esf.put(keyIterator.next(), emptyVector);
	}
	else
	*/    
	t.esf = getCurrentEvent().getEventSpecificFeatures();
    }    
        
    protected void esbdQ(EventTemplate t) {
	t.binaryEventData = getCurrentEvent().getEventSpecificBinaryDataAsByteArray();
    }
    
    protected void subeventQ(EventTemplate t) {
	
	subeventQ(t, getMatchMode());
    }    
        
    protected void subeventQ(EventTemplate t, int matchMode) {
	
	t.subEventMatchMode = matchMode;
	if (verbose)
	    System.out.print("...matchMode: " + t.subEventMatchMode + "...");
	// copy and add/subtract margin?
	t.subEventIDs = getCurrentEvent().getSubEventIDs();
    }    
    
    protected void addQueryType(EventTemplate t, int queryType) {
	
	switch (queryType) {
	    case 0: idQ(t, true); break;
	    case 1: idQ(t, false); break;
	    case 2: typeQ(t, true); break;
	    case 3: typeQ(t, false); break;
	    case 4: 
	    case 5: 
	    case 6: 	    
	    case 7: timeQ(t, queryType - 4); break;
	    case 8:
	    case 9:
	    case 10:
	    case 11: locationQ(t, queryType - 8); break;
	    case 12:
	    case 13:
	    case 14:
	    case 15: esfQ(t, queryType - 12, true); break;
	    case 16:
	    case 17:
	    case 18:
	    case 19: esfQ(t, queryType - 16, false); break;
	    case 20: esbdQ(t); break;
	    case 21: 
	    case 22: 
	    case 23: 
	    case 24: subeventQ(t, queryType - 21); break;
	    default: throw new RuntimeException("unknown query type");
	}
    }
    
    
    public BMQuery[] generateBMQueryArray() throws RuntimeException { 
	
	BMQuery[] bmq = new BMQuery[numberOfQueryTypes];
	// EventTemplate[] t = new EventTemplate[numberOfQueryTypes];
	
	Vector<Integer> v = new Vector<Integer>();
	for (int i = 0; i < bmq.length; i++)
	    v.add(i);
	
	for (int i = 0; i < bmq.length; i++) {
	    int x = rng.nextInt(v.size());
	    int vx = v.get(x);
	    
	    for (int iterations = 0; ; iterations++) {
		getNextEvent();	 
			
		// check: haveESF -> getCurrentEvent().hasEventSpecificFeatures()
		// and:   haveSubevent -> getCurrentEvent().hasSubEvents()
		if ((! (12 <= vx && vx <= 19) || getCurrentEvent().hasEventSpecificFeatures()) 
		    && (! (vx == 20) || getCurrentEvent().hasEventSpecificBinaryData())
		    && (! (21 <= vx && vx <= 24) || getCurrentEvent().hasSubEvents()))
		    break;
		
		if (iterations >= eventsInBuffer())
		    throw new RuntimeException("could not find suitable event for query construction " +
						bufferStats());
	    }
	    
	    if (verbose)
		System.out.print(" (" + vx + ") ");
	    bmq[i] = new BMQuery();
	    bmq[i].t = new EventTemplate();
	    addQueryType(bmq[i].t, vx);
	    bmq[i].src = getCurrentEvent().getEventID();
	    bmq[i].comment = "";
	    v.removeElementAt(x);
	}
	
	return bmq;
    }    
    
    // Generates an array of EventTemplates with only *ONE* "query component"
    public EventTemplate[] generateEventTemplateArray() throws RuntimeException { 

	EventTemplate[] t = new EventTemplate[numberOfQueryTypes];
	
	Vector<Integer> v = new Vector<Integer>();
	for (int i = 0; i < t.length; i++)
	    v.add(i);
	
	for (int i = 0; i < t.length; i++) {
	    int x = rng.nextInt(v.size());
	    int vx = v.get(x);
	    
	    for (int iterations = 0; ; iterations++) {
		getNextEvent();	 
			
		// check: haveESF -> getCurrentEvent().hasEventSpecificFeatures()
		// and:   haveSubevent -> getCurrentEvent().hasSubEvents()
    	        // and:   haveESBD -> getCurrentEvent().hasEventSpecificBinaryData()
		if ((! (12 <= vx && vx <= 19) || getCurrentEvent().hasEventSpecificFeatures()) 
		    && (! (vx == 20) || getCurrentEvent().hasEventSpecificBinaryData())
		    && (! (21 <= vx && vx <= 24) || getCurrentEvent().hasSubEvents()))
		    break;
		
		if (iterations >= eventsInBuffer())
		    throw new RuntimeException("could not find suitable event for query construction " +
					       bufferStats());
	    }
	    
	    if (verbose)
		System.out.print(" (" + vx + ") ");
	    t[i] = new EventTemplate();
	    addQueryType(t[i], vx);
	    v.removeElementAt(x);
	}
	
	return t;
    }
    
    public BMQuery generateBMQuery() throws RuntimeException { 
        return generateBMQuery(defMinQueryComponents, defMaxQueryComponents);
    }
    
    public BMQuery generateBMQuery(int minQueryComponents, int maxQueryComponents) throws RuntimeException { 
    
	BMQuery bmq = new BMQuery();
	bmq.t = generateEventTemplate(minQueryComponents, maxQueryComponents);
	bmq.src = getCurrentEvent().getEventID();
	bmq.comment = "";
	return bmq;
    }    

    public EventTemplate generateEventTemplate() throws RuntimeException { 
        return generateEventTemplate(defMinQueryComponents, defMaxQueryComponents);
    }
    
    public EventTemplate generateEventTemplate(int minQueryComponents, int maxQueryComponents) throws RuntimeException { 
	
	EventTemplate t = new EventTemplate();
	
	int n = minQueryComponents + rng.nextInt(maxQueryComponents - minQueryComponents + 1); // rng.nextInt(5); // (6);
	
	boolean haveID       = false;
	boolean haveType     = false;
	boolean haveTime     = false;
	boolean haveLocation = false;
	boolean haveESF      = false;
	boolean haveESBD     = false;
	boolean haveSubevent = false;
	
	getNextEvent();
	int i;
	for (i = 0; i < n; ) {
	    
	    double v = rng.nextDouble();
	    
	    // treat ID queries as a special case?
	    if (!haveID && v < pID) {
		if (verbose)
		    System.out.print("..id..");
		i++;
		haveID = true;
		// break; // ???
	    }
	    else if (!haveType && pID <= v && v < pType) {
		if (verbose)
		    System.out.print("..type..");
		i++;
		haveType = true;
	    }
	    else if (!haveTime && pType <= v && v < pTime) {
		if (verbose)
		    System.out.print("..time..");
		i++;
		haveTime = true;
	    }
	    else if (!haveLocation && pTime <= v && v < pLocation) {
		if (verbose)
		    System.out.print("..loc..");
		i++;
		haveLocation = true;
	    }
	    else if (!haveESF && pLocation <= v && v < pESF) {
		if (verbose)
		    System.out.print("..esf..");
		i++;
		haveESF = true;
	    }
	    else if (!haveESBD && pESF <= v && v < pESBD) {
		if (verbose)
		    System.out.print("..esbd..");
		i++;
		haveESBD = true;
	    }
	    else if (!haveSubevent && pESBD <= v && v < pSubevent) {
		if (verbose)
		    System.out.print("..se..");
		i++;
		haveSubevent = true;
	    } // superevent has the same structure as subevent
	    
	}
	
	// find an appropriate event to construct this query from...
	
	for (int iterations = 0; ; iterations++) {
	    // check: haveESF -> getCurrentEvent().hasEventSpecificFeatures()
	    // and:   haveSubevent -> getCurrentEvent().hasSubEvents()
	    // and:   haveESBD -> getCurrentEvent().hasEventSpecificBinaryData()
	    if ((!haveESF || getCurrentEvent().hasEventSpecificFeatures()) 
		&& (!haveSubevent || getCurrentEvent().hasSubEvents())
		&& (!haveESBD || getCurrentEvent().hasEventSpecificBinaryData()))
		break;
	    
	    if (iterations >= eventsInBuffer())
		throw new RuntimeException("could not find suitable event for query construction " +
		                           (haveESF ? " (haveESF) " : "") +
		                           (haveESF ? " (haveSubevent) " : "") +
		                           (haveESF ? " (haveESBD) " : "") +
					   bufferStats());
	    getNextEvent();	    
	}
	// construct query...
	if (haveID)
	    idQ(t);
	if (haveType)
	    typeQ(t);
	if (haveTime)
	    timeQ(t);
	if (haveLocation)
	    locationQ(t);
	if (haveESF)
	    esfQ(t);
	if (haveESBD)
	    esbdQ(t);
	if (haveSubevent)
	    subeventQ(t);
	
	if (verbose)
	    System.out.println("query should have " + i + " components ");
	if (i != n)
	    throw new RuntimeException("query constructed has wrong number of components (" + i 
	                                + "), should have " + n + " components!");
	    
	return t;
    }
    
    private String bufferStats() {
	
	int complex = 0, esfs = 0, esbd = 0;
	
	for (int iterations = 0; iterations < eventsInBuffer(); iterations++) {
	    if (getCurrentEvent().hasSubEvents())
		complex++;
	    if (getCurrentEvent().hasEventSpecificFeatures())
		esfs++;
	    if (getCurrentEvent().hasEventSpecificBinaryData())
		esbd++;
	    getNextEvent();
	}	
	
	String s = "(total events in buffer: " + eventsInBuffer() + 
		    ", complex: " + complex  + 
		    ", with ESFs: " + esfs +
		    ", with ESBD: " + esbd + 
		    ")";
	return s;
    }
    
    /*
    private int countComplex() {
	int c = 0;
	
	for (int iterations = 0; iterations < eventsInBuffer(); iterations++) {
	    if (getCurrentEvent().hasSubEvents())
		c++;
	    getNextEvent();
	}

	return c;
    }
    */
}
