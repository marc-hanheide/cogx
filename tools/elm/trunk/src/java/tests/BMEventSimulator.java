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

import elm.util.EventBuffer;


public class BMEventSimulator {

    // compile time constants
    public static final int bufferCapacity = 9;
    public static final int discard = 9;
    public static final int eventCounterMax = 9;

    
    public static final int dimension = 2;
    public static final double directionScalingFactor = 200.0;
    public static final double fieldScalingFactor = 1000.0;
    public static final long timeScalingFactor = 1000;
    public static final double ptBuffer = 20;
    
 /*   public static final String charField = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";*/
    public static final String esfCharField = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    public static final String typeCharField = "0123456789";
    public static final String stdCharField = typeCharField;


    
    public static final int maxEventTypeLength = 4;
    
//    public static final int esfKeyMax   = 100;
//    public static final int esfValueMax = 10000;
    
    public static final int esfKeyStringLength   = 4;
    public static final int esfValueStringLength = 6;
    
    public static final String esfKeyPrefix = "key-";
    public static final String esfValuePrefix = "";
    
    // (re-)configurable fields
    private boolean eventTypeWithPrefix = false;
    private int eventTypeLength = maxEventTypeLength;
    private int moveMode = 1;
    private int binaryEventDataSize = 100;
    private double pBinaryData = 1.0; // 0.2;
    private int eventTimeDiffMin = 1;
    private int eventTimeDiffMax = 1;
    private int minNumberOfESFs = 5;    
    private int maxNumberOfESFs = 5;
    private int minNumberOfESFValues = 1;
    private int maxNumberOfESFValues = 1;
    
    private EventBuffer buffer = new EventBuffer(bufferCapacity);
    private int eventCounter = 0;
    
    
    private Vector<PhysicalEntityID> entitiesInvolved = new Vector<PhysicalEntityID>();

    private double[] pos = new double[dimension];
    private double[] currentDirection = new double[dimension];
    private Random rng = new Random();
    private EventLocationFactory elFactory;

    public BMEventSimulator(EventLocationFactory elFactory) {
	
	this.elFactory = elFactory;
	
// 	entitiesInvolved.add(new PhysicalEntityID(10));
// 	entitiesInvolved.add(new PhysicalEntityID(11));
// 	entitiesInvolved.add(new PhysicalEntityID(12));

	for (int i = 0; i < currentDirection.length; i++)
	    currentDirection[i] = rng.nextDouble();
	
    }
    
    public void configure(boolean eventTypeWithPrefix,
			  int eventTypeLength,
			  int moveMode,
			  double pBinaryData,
			  int binaryEventDataSize,
			  int minNumberOfESFs,
			  int maxNumberOfESFs,
			  int minNumberOfESFValues,
			  int maxNumberOfESFValues) {
	
	this.eventTypeWithPrefix = eventTypeWithPrefix;
	this.eventTypeLength = eventTypeLength;
	this.moveMode = moveMode;
	this.pBinaryData = pBinaryData;
	this.binaryEventDataSize = binaryEventDataSize;
	this.minNumberOfESFs = minNumberOfESFs;
	this.maxNumberOfESFs = maxNumberOfESFs;	
	this.minNumberOfESFValues = minNumberOfESFValues;
	this.maxNumberOfESFValues = maxNumberOfESFValues;	
    }

    
    public EventLocationFactory getEventLocationFactory() {
	return elFactory;
    }

	
    public void move() {
	if (moveMode == 0)   // random jumps
	    random_move();
	else if (moveMode == 1) // rather continuous movements
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

    protected double[] getPosition() {
	return pos;
    }
    
    protected EventLocation getEventLocation() {
	return elFactory.fromPoint(pos, ptBuffer);
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

    // ---------------------------------------------------------------------------------
    
    protected int randInt(int min, int max) {
	int v = min + (max - min > 0 ? rng.nextInt(max - min + 1) : 0);
	if (v < min || v > max)
	    throw new RuntimeException("randInt failure");
	return v;
    }
    
    protected char getRandomChar() {
	
	return getRandomChar(stdCharField);
    }
    
    protected String getRandomString(int length) {
	
	return getRandomString(stdCharField, length);
    }
    
    protected char getRandomChar(String charField) {
	
	return charField.charAt(rng.nextInt(charField.length()));
    }
    
    protected String getRandomString(String charField, int length) {
	// TEMPORARY!!!
	StringBuffer sb = new StringBuffer();
	for (int i = 0; i < length; i++)
	    sb.append(getRandomChar(charField));
	return sb.toString();
    }
    
    
    protected EventType getEventType() {

	String typeString;
	
	if (eventTypeWithPrefix) 
	    typeString = "event-type-"  + rng.nextInt(1000);	   
	else
	    // ----------   TEMPORARY!!!  ----------------
	    typeString = "" + getRandomString(eventTypeLength);
	
	return new EventType(typeString);
    }
    
    // not all fields might contain valid values!
    /*
    protected static char[] getCharArray(int[] ind, int length) {
	
	char[] c = new char[length];
	for (int i = 0; i < length; i++)
	    c[i] = charField.charAt(ind[i]);
	return c;
    }
    */
    
    public static Vector<String> generateTypes() {
	
	return generateTypes("", maxEventTypeLength);	
    }    
    
    protected static Vector<String> generateTypes(String prefix, int length) {
		
	Vector<String> v = new Vector<String>();
	
	if (length == 1) 
	    for (int i = 0; i < typeCharField.length(); i++)
		v.add(prefix + typeCharField.charAt(i));
		  
	else if (length > 1)
	    for (int i = 0; i < typeCharField.length(); i++) {
		v.add(prefix + typeCharField.charAt(i));
		v.addAll(generateTypes(prefix + typeCharField.charAt(i), length-1));
	    }
	return v;	
    }    

    
    public static Vector<String[]> generateTypeHierarchy() {
	
	Vector<String[]> eth = new Vector<String[]>();
	
	Vector<String> types = generateTypes();
		
	for (String t : types) 
	    if (t.length() > 1) {
		String[] s = new String[2];
		s[0] = t;
		s[1] = t.substring(0, t.length() - 1);
		eth.add(s);
	    }
	
	return eth;	
    }
    
    protected EventTime getEventTime() {
	
	long microTimeEnd = System.currentTimeMillis();
	long microTimeStart = microTimeEnd - 
				timeScalingFactor * randInt(eventTimeDiffMin, eventTimeDiffMax);
	return new EventTime(microTimeStart, microTimeEnd);
    }
    
    public EventSpecificBinaryData getEventSpecificBinaryData() throws IOException {
	if (rng.nextDouble() < pBinaryData)
	    return (binaryEventDataSize == 0 ? null : new BMEventSpecificBinaryData(binaryEventDataSize));
	return null;
    }
    
    protected String getRandomESFValue() {
	// ----------   TEMPORARY!!!  ----------------
	// return esfValuePrefix + rng.nextInt(esfValueMax);
	return esfValuePrefix + getRandomString(esfCharField, esfValueStringLength);

    }
    
    protected String getRandomESFKey() {
	// ----------   TEMPORARY!!!  ----------------
	// return esfKeyPrefix + rng.nextInt(esfKeyMax);
	return esfKeyPrefix + getRandomString(esfCharField, esfKeyStringLength);
    }
    
    protected EventSpecificFeatures getESFs() {

	int numberOfESFs = randInt(minNumberOfESFs, maxNumberOfESFs);
	
	if (numberOfESFs == 0)
	    return null;
	
	EventSpecificFeatures esf = new EventSpecificFeatures();
		
	for (int i = 0; i < numberOfESFs; i++) {
	    Vector<String> v = new Vector<String>();
	    int numberOfESFValues = randInt(minNumberOfESFValues, maxNumberOfESFValues);
	    for (int j = 0; j < numberOfESFValues; j++)
		v.add(getRandomESFValue());
	    esf.put(getRandomESFKey(), new HashSet<String>(v));
	}
	return esf;
    }

    public Event moveAndGenerateAtomicEvent() throws EventIDException, IOException {
	
	move();
	return new AtomicEvent(getEventType(), 
			       getEventTime(), 
			       getEventLocation(), 
			       getEventSpecificBinaryData(), 
			       entitiesInvolved, 
			       getESFs());
    }
    
    public Event generateEvent() throws EventIDException, IOException, elm.util.CircularBufferException {
	
	if (eventCounter >= eventCounterMax) {
	    eventCounter = 0;
	    ComplexEvent newEvent = new ComplexEvent(getEventType(), 
				 		     buffer.getRange(0, buffer.size() - 1),
						     getESFs());
	    newEvent.setEventSpecificBinaryData(getEventSpecificBinaryData());
	    int d = buffer.size() < discard ? buffer.size() : discard;
	    buffer.discard(d);
	    buffer.addOrOverwrite(newEvent);
	    return newEvent;
	}
	else {
	    eventCounter++;
	    Event e = moveAndGenerateAtomicEvent();
	    buffer.addOrOverwrite(e);
	    return e;    
	}
    }
    
    /**
      *  Accepts externally generated events and returns "recognized" complex events
      *  occasionally (or null).
      *  Generated events are *NOT* stored in the internal buffer!!! 
      *  Another call to this method is expected for this.
      */
    public Event processExternallyGeneratedEvent(Event event) throws EventIDException, IOException, elm.util.CircularBufferException {
	
	if (eventCounter >= eventCounterMax) {
	    eventCounter = 0;
	    ComplexEvent newEvent = new ComplexEvent(getEventType(), 
				 		     buffer.getRange(0, buffer.size() - 1),
						     getESFs());
	    newEvent.setEventSpecificBinaryData(getEventSpecificBinaryData());
	    int d = buffer.size() < discard ? buffer.size() : discard;
	    buffer.discard(d);
	    // buffer.addOrOverwrite(newEvent);
	    return newEvent;
	}
	else {
	    eventCounter++;
	    buffer.addOrOverwrite(event);
	    return null;    
	}
    }
}
