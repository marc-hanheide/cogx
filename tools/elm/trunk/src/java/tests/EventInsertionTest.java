/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.util.Vector;

import elm.dbio.*;
import elm.event.*;
import elm.eventrecognition.*;

public class EventInsertionTest {

    static final int simMode = 2;
    static final int trials = 1000;
    static final int subEventsSize = 10;
    static final int drift = 3;
    static int   eventCnt = 0;
    static Event[] events = new Event[trials];
    static Vector<Event> subEvents = null;
    // static long[] entitiesInvolved = new long[] { 10, 11, 12 };

    static Vector<PhysicalEntityID> entitiesInvolved = new Vector<PhysicalEntityID>();

    static boolean dbBenchmark = false;
    static String benchmarkConfigName = "testconfig1";
    

    public static Connection connect() {
	Connection connection = null;

	try {
	    Class.forName("org.postgresql.Driver");
	}
	catch(ClassNotFoundException cnfe) {
	    System.out.println("Couldn't find the driver!");
	    cnfe.printStackTrace();
	    System.exit(1);
	}

	System.out.print("Registered the driver.\nConnecting to the database... ");

	try {
	    //The second and third arguments are the username and password,
	    //respectively.They should be whatever is necessary to connect
	    // to the database.
	    connection = DriverManager.getConnection("jdbc:postgresql://localhost/elm",
						     "elm", "somepw");
	}
	catch(SQLException se) {
	    System.out.println("Couldn't connect.");
	    se.printStackTrace();
	    System.exit(1);
	}
              
	System.out.println("done.");
                
	return connection;
    }

    static Vector<Event> getSubEvents() {
	int size = (eventCnt > subEventsSize ? subEventsSize : eventCnt);
	subEvents = new Vector<Event>(size);	
	for (int i = 0; i < size; i++)
	    subEvents.add(events[eventCnt - i - 1]);
	return new Vector<Event>(subEvents);
    }

    static Vector<PhysicalEntityID> getObjectsInvolved() {
	return entitiesInvolved;
    }

       

    public static void main(String[] args) {

	try {
	    entitiesInvolved.add(new PhysicalEntityID(10));
	    entitiesInvolved.add(new PhysicalEntityID(11));
	    entitiesInvolved.add(new PhysicalEntityID(12));


	    long time_m = 0, time_e = 0, time_e2, time_m1, time_m2;
				
	    Event e = null;
	    EventSimulator sim = new EventSimulator();
	    EventInserter  ins = new EventInserter(connect());

	    if (dbBenchmark)
		ins.turnOnDBBenchmark(benchmarkConfigName);

	    RecognitionManager recManager = null;
	    if (simMode == 2) {
		recManager = new RecognitionManager();
		recManager.start();
		recManager.registerRecognizer(new DummyRecognizer(ins, recManager));
	    }

	    for (eventCnt = 0; eventCnt < trials; eventCnt++) {
		time_m1 = System.currentTimeMillis();
		if (simMode == 0) {
		    sim.move();
		    time_m2 = System.currentTimeMillis();
		       
		    events[eventCnt] = ins.storeEvent("type_prefix_ABC", 
						      sim.getTime(),
						      sim.getTime(),
						      sim.getEventLocation(),
						      sim.getEventSpecificBinaryData(),
						      getSubEvents(),
						      getObjectsInvolved());
		}
		else if (simMode == 1) {
		    e = sim.moveAndGetEvent();
		    time_m2 = System.currentTimeMillis();
		    // System.out.println("about to store:\n" + e);
		    ins.storeEvent(e);
		}
		else if (simMode == 2) {
		    e = sim.moveAndGetAtomicEvent();
		    time_m2 = System.currentTimeMillis();
		    // System.out.println("about to store:\n" + e);
		    ins.storeEvent(e);
		    recManager.newEvent(e);
		}
		else
		    throw new Exception("undefined simMode");
		   
		time_e2 = System.currentTimeMillis();
		time_m += time_m2 - time_m1;
		time_e += time_e2 - time_m2;
	    }


	    if (simMode == 2) {
		System.out.print("waiting for recognizers to finish... ");
		recManager.interrupt();
		recManager.finishQueue();
		System.out.println("done.");
	    }
				
	    System.out.println("\ngenerated " + eventCnt + " events");
	    System.out.println("time_m: " + time_m + "\ntime_e: " + time_e + 
			       " (not containing recognizers' store calls!)");

	    if (dbBenchmark)
		System.out.println("\nsee database (config name: " + 
				   benchmarkConfigName + 
				   ") for more details.");
	    else {
		System.out.println("\nmore detailed:\nstored events (total): " 
				   + ins.storedEvents +
				   "\ninStoreEventsTime: " + ins.inStoreEventTime);
		
		System.out.println("newEvent: " + ins.newEventTime);
		// System.out.println("ettTime: " + ins.ettTime);
		System.out.println("objInvTime: " + ins.objInvTime);
		System.out.println("elmTime: " + ins.elmTime);
		System.out.println("binaryEventDataTime: " + ins.binaryEventDataTime);
		System.out.println("esfTime: " + ins.esfTime);
		System.out.println("seTime: " + ins.seTime);
		System.out.println("apexTime: " + ins.apexTime);
		System.out.println("commitTime: " + ins.commitTime);
	    }

	}
	catch (Exception e) {
	    e.printStackTrace();
	}

    }

}
