package celmarchitecture.subarchitectures.recognition;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import elm.event.*;
import elm.eventrecognition.*;
import elm.util.EventBuffer;
import elm.util.CircularBufferException;

import celmarchitecture.global.EventTypes;
import celmarchitecture.global.EventTypeNames;
import celmarchitecture.global.GlobalSettings;


/**
 *  UNDER CONSTRUCTION
 *  @author Dennis Stachowicz
 */
public class BeWelcomedRecognizer implements SuperEventRecognizer {

    // size of buffer: should be enough to store the possible subevents.
    // Exceptions will show that it is too small otherwise.
    public static final int bufferCapacity  = 1000;

    // maximum time between entering and being greeted: 15 seconds
    public static final long maxWaitingTime = 15000;

    // maximum lag time expected for events to end up here
    // i.e. lags due to CAST, storage or other processing
    public static final long lagTime        = 15000;

    private final Lock lock                 = new ReentrantLock();

    private EventBuffer buffer              = new EventBuffer(bufferCapacity);

    private Recognizer castRecognizer       = null;

    private long    enteringTime            = 0;
    private boolean haveEnterEvent          = false;
    private boolean haveGreetingEvent       = false;
    // private boolean personDetected          = false;
    
    public BeWelcomedRecognizer(Recognizer r) {
	castRecognizer = r;
    }

    public void newEvent(Event event) {

	lock.lock();

	try {

	    if (event.getEventType().isName(EventTypeNames.enterRoom)) {

		processBufferedEvents();
		
		enteringTime = event.getTime().getMicroTimeEnd();
		buffer.flush();
		buffer.add(event);
		haveEnterEvent = true;
		haveGreetingEvent = false;

	    }

	    else if (haveEnterEvent) {

		if (event.getTime().getMicroTimeBegin() - enteringTime < maxWaitingTime) {
		    
		    if (event.getEventType().isName(EventTypeNames.dialogueMoveOPENING)) {
		
			buffer.add(event);
			haveGreetingEvent = true;
		    }
		    else if (event.getEventType().isName(EventTypeNames.beApproached))
			buffer.add(event);
		}

				
		if (System.currentTimeMillis() > enteringTime + maxWaitingTime + lagTime) 		
			processBufferedEvents();		    
		    	
	    }
			    
	}
	catch (CircularBufferException ex) {
	    ex.printStackTrace();
	    
	    if (GlobalSettings.exitOnException)
		System.exit(1);
	}
	catch (EventIDException ex) {
	    ex.printStackTrace();
	    
	    if (GlobalSettings.exitOnException)
		System.exit(1);
	}
	finally {
	    lock.unlock();
	}

    }

    protected void reset() throws CircularBufferException {

	lock.lock();

	try {

	    buffer.flush();
	    haveEnterEvent = false;
	    haveGreetingEvent = false;
	    // personDetected = false;
	    enteringTime = 0;

	}
	finally {
	    lock.unlock();
	}

    }

    protected void processBufferedEvents() throws CircularBufferException, EventIDException {
	
	lock.lock();

	try {

	    if (haveEnterEvent) {
		
		if (haveGreetingEvent)	{	    
		    ComplexEvent newEvent = 
			new ComplexEvent(EventTypes.beWelcomedUponEntry, 
					 buffer.getRange(0, buffer.size() - 1)); 
		    castRecognizer.newEventDetected(newEvent);
		}
		/*
		else if (personDetected) { // we cannot be ignored if there is nobody to ignore us
		    ComplexEvent newEvent = 
			new ComplexEvent(EventTypes.beIgnoredUponEntry, 
					 buffer.getRange(0, buffer.size() - 1));
		    castRecognizer.newEventDetected(newEvent);
		}
		*/
		
	    }
	    
	    reset();
	}
	finally {
	    lock.unlock();
	}

    }

}

