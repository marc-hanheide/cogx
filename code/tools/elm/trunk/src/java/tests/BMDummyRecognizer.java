/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import java.util.Random;
import java.util.Vector;

import elm.event.*;
import elm.eventrecognition.*;
import elm.dbio.ELMDatabaseWriter;
import elm.util.EventBuffer;

public class BMDummyRecognizer implements SuperEventRecognizer {

    public static final int bufferCapacity = 10;
    public static final int minSize = 5;
    public static final int maxSize = 10;
    public static final int discard = 10;
    public static final double pCoin = 0.5;
    // public static final String eventTypeStringComplex = "dummy_complex_type";

    private EventBuffer buffer = new EventBuffer(bufferCapacity);
    // private EventType etypeComplex = new EventType(eventTypeStringComplex);

    private Random rng = new Random();

    private ELMDatabaseWriter dbWriter = null;
    private RecognitionManager manager = null;
    private BMEventSimulator eventSimulator = null;
    private BMQuerySimulator querySimulator = null;

    public BMDummyRecognizer(ELMDatabaseWriter w, 
			     RecognitionManager m,
			     BMEventSimulator eventSim,
			     BMQuerySimulator querySim) {
	dbWriter = w;
	manager = m;
	eventSimulator = eventSim;
	querySimulator = querySim;
    }

    public void newEvent(Event event) {

	
	try {
	    
	    buffer.addOrOverwrite(event);
	    if ((buffer.size() > minSize && flipCoin()) || buffer.size() >= maxSize) {

		ComplexEvent newEvent = new ComplexEvent(eventSimulator.getEventType(), 
							 buffer.getRange(0, buffer.size() - 1),
							 eventSimulator.getESFs());
		int d = buffer.size() < discard ? buffer.size() : discard;
		buffer.discard(d);
		dbWriter.storeEvent(newEvent);
		manager.newEvent(newEvent);
		querySimulator.newStoredEvent(newEvent);
	    }

	    
	}
	catch (Exception ex) {
	    ex.printStackTrace();
	}
    }


    private boolean flipCoin() {
	if (rng.nextDouble() < pCoin)
	    return true;
	return false;
    }
}
