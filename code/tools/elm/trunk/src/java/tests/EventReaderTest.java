/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;

import java.util.Vector;
import java.util.Date;
import java.text.SimpleDateFormat;

import elm.dbio.*;
import elm.event.*;

public class EventReaderTest {

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

	System.out.println("Connecting...");

	try {
	    connection = DriverManager.getConnection("jdbc:postgresql://localhost/elm",
						     "elm", "somepw");
	}
	catch(SQLException se) {
	    System.out.println("Couldn't connect.");
	    se.printStackTrace();
	    System.exit(1);
	}

	if (connection != null)
	    System.out.println("Connected to the database.");

	return connection;
    }


    protected static void testIDRange(ELMDatabaseReader dbReader, long minID, long maxID) throws Exception {

	java.util.Vector<Event> events = dbReader.getEventsByIDRange(minID, maxID);
	for (int i = 0; i < events.size(); i++) {
	    Event event = events.get(i);
	    System.out.println("\n\ngot:\n" + event); 
		    
	}
    }

    protected static void testSuperIDs(ELMDatabaseReader dbReader, long eventID) throws Exception {

	java.util.Vector<EventID> superEventIDs = dbReader.getSuperEventIDs(new EventID(eventID));
	System.out.println("\n\ngot:\n" + superEventIDs); 
			
    }

   //  protected static void testEventSpecificBinaryData(ELMDatabaseReader dbReader, long eventID) throws Exception {

// 	EventSpecificBinaryData data = dbReader.getEventSpecificBinaryData(new EventID(eventID));
// 	System.out.println("\n\ngot:\n" + data); 
			
//     }
       

    protected static void testTemplate(ELMDatabaseReader dbReader,
				       EventLocationFactory elFactory) throws Exception {

	EventTemplate template = new EventTemplate();

	template.minEventID = new EventID(1);
	
	// template.apex = template.boolFalse;

	// template.minDegree = 1;
	// template.maxDegree = 3;

	// template.eventType = new EventType("action");
	// template.exactTypeMatch = false;

// 	SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
// 	template.time = new EventTime(df.parse("2008-08-06 15:10:07.502"),
// 				      df.parse("2008-08-06 15:10:07.506"));
// 	template.timeMatchMode = template.matchIntersectionNotEmpty;

	// double[] pt = new double[] { -425.0, -120.0 };

	// template.location = elFactory.fromPoint(pt, 1000);
	// template.locationMatchMode = template.matchExact;
	// template.locationMatchMode = template.matchSubset;
	// template.locationMatchMode = template.matchIntersectionNotEmpty;

	// template.location = elFactory.fromPoint(pt, 0.1);
	// template.locationMatchMode = template.matchSuperset;

	
	// template.binaryEventData = EventSpecificBinaryDataIO.toByteArray(new EventSpecificBinaryDataTest(0));
// 	Vector<EventID> ev = new Vector<EventID>();
// 	for (int i = 24; i < 25; i++)
// 	    ev.add(new EventID(i));

// 	template.subEventIDs = ev;
// 	// template.subEventMatchMode = template.matchIntersectionNotEmpty;
// 	// template.subEventMatchMode = template.matchSuperset;
//  	template.subEventMatchMode = template.matchExact;


	// template.superEventIDs = ev;
	// template.superEventMatchMode = template.matchIntersectionNotEmpty;
	// template.superEventMatchMode = template.matchSubset;
	// template.superEventMatchMode = template.matchSuperset;
	// template.superEventMatchMode = template.matchExact;

	/*
	template.physicalEntityIDs = new Vector<PhysicalEntityID>();
	template.physicalEntityIDs.add(new PhysicalEntityID(10));
	template.physicalEntityIDs.add(new PhysicalEntityID(11));
	template.physicalEntityIDs.add(new PhysicalEntityID(12));
	template.physicalEntityMatchMode = template.matchExact;
	*/

	// java.util.Vector<Event> events = dbReader.getMatchingEvents(template, 50, 0);

	

	template.esf = new EventSpecificFeatures();
/*	
	// template.esf.addKeyValuePair("key 0", "15");
	// no. 3:
	template.esf.addKeyValuePair("key 0", "66");
	template.esf.addKeyValuePair("key 1", "11");
	template.esf.addKeyValuePair("key 1", "66");
	template.esf.addKeyValuePair("key 2", "22");
	template.esf.addKeyValuePair("key 2", "11");
	template.esf.addKeyValuePair("key 2", "66");	
	// template.esf.addKeyValuePair("key 2", "1001");
*/	
	// template.esfMatchMode = template.matchSuperset; 
	// template.esfMatchMode = template.matchSubset; 
	template.esfMatchMode = template.matchIntersectionNotEmpty; // runs, further testing needed
	// template.esfMatchMode = template.matchExact;
	template.esfRestrictMatchToKeys = false;

	
	int max = 100;
	java.util.Vector<Event> events = 
	    dbReader.getMatchingEvents(template, max, 0, 
				       ELMDatabaseReader.orderModeTimeOnly);

	for (int i = 0; i < events.size(); i++) 
	    System.out.println("\n\ngot (" + (i + 1) + "):\n" + events.get(i)); 
	System.out.println("\n\n(retrieved " + events.size() + 
			   " in total, pre-set maximum was " + max + " events)\n");
	
    }

    public static void main(String[] args) {

	try {
	    EventLocationFactory elFactory = new EventLocationFactory();

	    ELMDatabaseReader dbReader = new ELMDatabaseReader(connect(), elFactory);

	    dbReader.setDebugMode(true);
	    // dbReader.turnOnDBBenchmark("test-query-config", null);

	    testTemplate(dbReader, elFactory);

	    // testIDRange(dbReader, 0, 100000);
	    // testIDRange(dbReader, 362, 365);

	    // testSuperIDs(dbReader, 3);
	    // testEventSpecificBinaryData(dbReader, 3);

	}
	catch (Exception e) {
	    e.printStackTrace();
	}

    }

}
