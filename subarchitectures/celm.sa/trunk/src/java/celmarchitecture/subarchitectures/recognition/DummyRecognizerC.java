package celmarchitecture.subarchitectures.recognition;

import java.util.Random;
import java.util.Vector;

import elm.event.*;
import elm.eventrecognition.*;
import elm.dbio.ELMDatabaseWriter;
import elm.util.EventBuffer;

import celmarchitecture.global.GlobalSettings;


/** 
 *  This class is just for test the information flow in the recognition subsystem.
 *  It combines random sequences of events to one complex event.
 *  @author Dennis Stachowicz
 */
public class DummyRecognizerC implements SuperEventRecognizer {

    public static final int bufferCapacity = 10;
    public static final int minSize = 5;
    public static final int discard = 7;
    public static final double pCoin = 0.2;
    public static final String eventTypeStringComplex = "dummy complex type 1";

    private EventBuffer buffer = new EventBuffer(bufferCapacity);
    // private Vector<Event> ev = new Vector<Event>();

    // private ELMTypeHierarchy eth = new ELMTypeHierarchy();
    private EventType etypeComplex = new EventType(eventTypeStringComplex);

    private Random rng = new Random();

    private Recognizer castRecognizer  = null;

    public DummyRecognizerC(Recognizer r) {
	castRecognizer = r;
    }

    public void newEvent(Event event) {

	// TEMPORARY!!!
	try {

	    /*   to test de-serialisation:
	    if (event.hasEventSpecificBinaryData()) {
		Object obj = event.getEventSpecificBinaryDataAsRawObject();
		if (obj instanceof Person) {
		    Person p = (Person) obj;
		    System.out.println("\n\n\n-----------------DummyRec.: Person id: " + p.m_id
				       + "----------------\n\n\n");
		}
		else if (obj instanceof RobotPose) {
		    RobotPose p = (RobotPose) obj;
		    System.out.println("\n\n\n-----------------DummyRec.: RobotPose x: " + p.m_x
				       + "----------------\n\n\n");
		}
	    }	    
	    */

	    buffer.addOrOverwrite(event);
	    if (buffer.size() > minSize && flipCoin()) {
		/*
		Event[] gr = buffer.getRange(0, buffer.size() - 1);
		System.err.println("buffer.getRange().length: " + gr.length);
		if (gr.length < bufferCapacity)
		    for (int i = 0; i < gr.length; i++)
			System.err.println("buffer.getRange() [" + i + "]: " + gr[i]);
		*/
		ComplexEvent newEvent = new ComplexEvent(etypeComplex, 
							 buffer.getRange(0, buffer.size() - 1));
		int d = buffer.size() < discard ? buffer.size() : discard;
		buffer.discard(d);
		castRecognizer.newEventDetected(newEvent);
	    }
	
	    /*
	    ev.add(event);
	    if (ev.size() > minSize && flipCoin()) {
		Vector<Event> subEvents = new Vector<Event>(ev.subList(0, ev.size()));
		ComplexEvent newEvent = new ComplexEvent(etypeComplex, 
							 subEvents);
		dbWriter.storeEvent(newEvent);
		manager.newEvent(newEvent);
	    }
	    */
	    
	}
	catch (Exception ex) {
	    ex.printStackTrace();
	    
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
	}
    }


    private boolean flipCoin() {
	if (rng.nextDouble() < pCoin)
	    return true;
	return false;
    }
}
