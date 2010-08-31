/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.eventrecognition;

import java.util.Vector;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ArrayBlockingQueue;

import elm.event.Event;

public class RecognitionManager extends Thread {

    public static final int defaultBufferSize = 1024;
    // private EventBuffer buffer = null;

    // might be changed to a priority queue later to reflect temporal order!!!
    private BlockingQueue<Event> buffer = null; 

    private Vector<SuperEventRecognizer> recognizers = new Vector<SuperEventRecognizer>();
    

    public RecognitionManager() {
	// buffer = new EventBuffer(defaultBufferSize);
	buffer = new ArrayBlockingQueue<Event>(defaultBufferSize);
    }

    public RecognitionManager(int bufferSize) {
	// buffer = new EventBuffer(bufferSize);
	buffer = new ArrayBlockingQueue<Event>(bufferSize);
    }
	
    public void registerRecognizer(SuperEventRecognizer ser) {

	synchronized(recognizers) {
	    if (!recognizers.contains(ser))
		recognizers.add(ser);
	}
    }

    public void removeRecognizer(SuperEventRecognizer ser) {

	synchronized(recognizers) {
	    recognizers.remove(ser);
	}
    }

    public void newEvent(Event e) throws IllegalStateException { // EventBufferException {

	buffer.add(e);
    }

    public void run() {

	Event event = null;

	try {
	    while (true) {
		event = buffer.take();
		synchronized(recognizers) {
		    for (int i = 0; i < recognizers.size(); i++) 
			recognizers.get(i).newEvent(event);
		}
	    }
	}
	catch (InterruptedException e) {
	}
    }

    public void finishQueue() {

	Event event = null;
	
	while (true) {

	    event = buffer.poll();
	    if (event == null)
		break;

	    synchronized(recognizers) {
		    for (int i = 0; i < recognizers.size(); i++) 
			recognizers.get(i).newEvent(event);
	    }
	    
	}
	
    }
    
}
