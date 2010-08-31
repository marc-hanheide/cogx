/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tools;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;

import java.util.Vector;
import java.util.Date;
import java.text.SimpleDateFormat;

import elm.dbio.*;
import elm.event.*;

public class PrintEvent {

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

    public static void printUsage(boolean exit) {
	
	System.out.println("usage:\n" + 
			   "      java elm.tools.PrintEvent <eventID>         (to print information on event with ID <eventID>)\n" + 
			   "or:   java elm.tools.PrintEvent  <minID> <maxID>  (to print information on events with IDs between <minID> and <maxID>)");
	
	if (exit)
	    System.exit(1);
    }

    public static void main(String[] args) {

	try {
	    int min = 0, max = 0; 
	    
	    if (args.length == 1) 
		min = max = Integer.parseInt(args[0]);	    
	    else if (args.length == 2) {
		min = Integer.parseInt(args[0]);
		max = Integer.parseInt(args[1]);
	    }
	    else
		printUsage(true);
	    
	    EventLocationFactory elFactory = new EventLocationFactory();
	    ELMDatabaseReader dbReader = new ELMDatabaseReader(connect(), elFactory);
	    
	    java.util.Vector<Event> events = dbReader.getEventsByIDRange(min, max);
	    if (events.size() == 0)
		System.out.println("Sorry, I did not find any event with the specified ID(s).");
	    else
		for (int i = 0; i < events.size(); i++) 
		    System.out.println("\n\n" + events.get(i).eventToString(true));

	}
	catch (Exception e) {
	    e.printStackTrace();
	}

    }

}