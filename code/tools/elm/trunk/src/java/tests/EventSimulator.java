/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import java.util.Random;
import java.util.Date;
import java.util.Vector;
import java.util.HashSet;

import java.io.IOException;

import elm.event.*;

public class EventSimulator {

    static final int move_mode = 1;
    static final int binaryEventDataTestSize = 0; // make all equal for the moment... 5;
    static final int dimension = 2;
    static final double directionScalingFactor = 200.0;
    static final double fieldScalingFactor = 1000.0;
    static final long timeScalingFactor = 1000;
    // static final String eventTypeString = "type_prefix_ABC";
    // static final String eventTypeStringAtomic = "look at";
    // static final String eventTypeStringComplex = "take a look around";

    static final double ptBuffer = 20;
    
    Vector<PhysicalEntityID> entitiesInvolved = new Vector<PhysicalEntityID>();

    static final boolean useArray = false;
    // static final int eventCntMin = 4;
    static final int eventCntMax = 8;
    Event[] eventArray = new Event[eventCntMax];
    Vector<Event> eventVector = new Vector<Event>();
    int eventCnt = 0;

    double[] pos = new double[dimension];
    double[] currentDirection = new double[dimension];
    Random rng = new Random();
    EventLocationFactory elFactory = new EventLocationFactory();

    public EventSimulator() {
	
	entitiesInvolved.add(new PhysicalEntityID(10));
	entitiesInvolved.add(new PhysicalEntityID(11));
	entitiesInvolved.add(new PhysicalEntityID(12));

	for (int i = 0; i < currentDirection.length; i++)
	    currentDirection[i] = rng.nextDouble();
	
    }
    
    public EventLocationFactory getEventLocationFactory() {
	return elFactory;
    }

	
    public void move() {
	if (move_mode == 0)   // random jumps
	    random_move();
	else if (move_mode == 1) // rather continuous movements
	    cont_move();	
    }

    void cont_move() {
	// change direction a little bit...
	for (int i = 0; i < currentDirection.length; i++)
	    currentDirection[i] = (rng.nextDouble() - 0.5) * directionScalingFactor;


	for (int i = 0; i < pos.length; i++)
	    pos[i] += currentDirection[i];
    }

    void random_move() {
	for (int i = 0; i < pos.length; i++)
	    pos[i] = rng.nextDouble() * fieldScalingFactor;
    }	


    public double[] getPosition() {
	return pos;
    }
    public EventLocation getEventLocation() {
	return elFactory.fromPoint(pos, ptBuffer);
    }

    public long getTime() {
	return System.currentTimeMillis();
    }

    public EventSpecificBinaryData getEventSpecificBinaryData() {
	return new EventSpecificBinaryDataTest(binaryEventDataTestSize);
    }

    public EventType getEventType(boolean atomic) {

	String typeString;
	if (atomic)
	    typeString = "atomic-type-"  + rng.nextInt(100);
	else
	    typeString = "complex-type-" + rng.nextInt(100); 
	return new EventType(typeString);
    }

    public Event moveAndGetAtomicEvent() throws EventIDException, IOException {

	long microTimeBegin = getTime();

	EventTime time = new EventTime(microTimeBegin, getTime());
	
	move();

	return new AtomicEvent(getEventType(true), time, getEventLocation(), 
			       getEventSpecificBinaryData(), entitiesInvolved, getAssociatedData());

    }

    protected EventSpecificFeatures getAssociatedData() {

	Vector<String> v = new Vector<String>();
	EventSpecificFeatures esf = new EventSpecificFeatures();
	for (int i = 0; i < 3; i++) {
	    v.add("" + rng.nextInt(100));
	    esf.put("key " + i, new HashSet<String>(v));
	}
	return esf;
    }

    public Event moveAndGetEvent() throws EventIDException, IOException {

	long microTimeBegin = getTime();
	EventTime time = new EventTime(microTimeBegin, getTime());
	Event event = null;

	move();

	// test array implementation
	if (useArray) {

	    if (eventCnt == eventArray.length) {
		eventCnt = 0;
		event = new ComplexEvent(getEventType(false), eventArray);
	    }
	    else {
		event = new AtomicEvent(getEventType(true), time, getEventLocation(), getEventSpecificBinaryData(), entitiesInvolved, getAssociatedData());
		eventArray[eventCnt++] = event;
	    }
	}
	// test Collection implementation
	else {
	   
	    if (eventCnt == eventCntMax) {
		event = new ComplexEvent(getEventType(false), eventVector);
		eventVector.clear();
		// eventVector.add(event);
		eventCnt = 0;
	    }
	    else {
		event = new AtomicEvent(getEventType(true), time, getEventLocation(), getEventSpecificBinaryData(), entitiesInvolved, getAssociatedData());
		eventVector.add(event);
		eventCnt++;
	    }

	}
	return event;
    }
    
    public void printPositionAndTime() {
	for (int i = 0; i < pos.length; i++)
	    System.out.print(pos[i] + " ");
	// Date date = new Date();
	long currentTime = System.currentTimeMillis(); 
	System.out.println("\n at time " 
			   + currentTime / timeScalingFactor + "." 
			   + currentTime % timeScalingFactor);
	// old junk...
	// + date);
	// SimpleDateFormat formatter = new SimpleDateFormat("yyyy:MM:dd:HH:mm:ss.SSS");
    }

}
