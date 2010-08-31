/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.eventrecognition;

import java.util.Random;
import java.util.Vector;

import elm.event.*;
import elm.dbio.ELMDatabaseWriter;
import elm.util.EventBuffer;

/**
 *  DummyRecognizer is a class to test the event recognition interfaces.
 *  It stores all incoming events in a circular buffer (EventBuffer). If at
 *  least minSize events are available it flips a(n unfair) coin to 
 *  decide whether to construct and report a complex event from the events available.
 *  Then it discards a certain number of events (or less if less are available).
 */
public class DummyRecognizer implements SuperEventRecognizer {

    public static final int bufferCapacity = 20;
    public static final int minSize = 5;
    public static final int discard = 7;
    public static final double pCoin = 0.2;
    public static final String eventTypeStringComplex = "dummy_complex_type";

    private EventBuffer buffer = new EventBuffer(bufferCapacity);
    private EventType etypeComplex = new EventType(eventTypeStringComplex);

    private Random rng = new Random();

    private ELMDatabaseWriter dbWriter = null;
    private RecognitionManager manager = null;

    public DummyRecognizer(ELMDatabaseWriter w, RecognitionManager m) {
	dbWriter = w;
	manager = m;
    }

    public void newEvent(Event event) {

	
	try {
	    
	    buffer.addOrOverwrite(event);
	    if (buffer.size() > minSize && flipCoin()) {

		ComplexEvent newEvent = new ComplexEvent(etypeComplex, 
							 buffer.getRange(0, buffer.size() - 1));
		int d = buffer.size() < discard ? buffer.size() : discard;
		buffer.discard(d);
		dbWriter.storeEvent(newEvent);
		manager.newEvent(newEvent);
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
