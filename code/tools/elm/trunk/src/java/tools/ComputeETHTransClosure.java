/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tools;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;

import elm.dbio.*;
import elm.event.*;
import elm.types.*;

public class ComputeETHTransClosure {

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



    public static void main(String[] args) {

	try {
	    EventTypeHierarchyIO ethIO = new EventTypeHierarchyIO(connect());

	    
	    System.out.print("loading type hierarchy...");
	    TypeHierarchy<EventType> eth = ethIO.loadSourceTypeHierarchy();
	    // System.err.println("eth: " + eth);
	    System.out.print(" done.\ncomputing transitive closure and storing...");
	    ethIO.computeAndStoreTransitiveClosure(eth);
	    System.out.print(" done.\n");
	    
	    /*
	    System.out.print("load, compute transitive closure, store... ");
	    ethIO.loadComputeTCStore();
	    System.out.print(" done.\n");
	    */
	}
	catch (Exception e) {
	    e.printStackTrace();
	}

    }

}
