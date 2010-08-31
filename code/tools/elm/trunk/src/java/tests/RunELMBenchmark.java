/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.tests;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;
import java.sql.Statement;
import java.sql.PreparedStatement;
import java.util.Vector;

import elm.dbio.*;
import elm.dbio.names.*;
import elm.event.*;
import elm.eventrecognition.*;
import elm.types.*;


public class RunELMBenchmark {

    public static final int sleepTime = 300000; // 5 min. sleep between runs
    public static final int firstRunNo = 1;
    public static final int lastRunNo = 10;
    public static final int rounds = 1000;  // 1000x100 -> 100,000 events per run
    public static final int insertionsPerRound = 100;
    public static final int queryMode = 2; // 1: queriesInSets (uni-queries), 2: randomQueries (possibly multi-queries)
    public static final int querySetsPerRound = 1;  // for runQueriesInSets() 
    public static final int queriesPerRound = 25;   // for runRandomQueries() 
    public static final int queryLimit = 1;
    public static final int queryOrderMode = ELMDatabaseReader.orderModeNone;// ELMDatabaseReader.orderModeTimeOnly;
    
    public static final boolean verbose = false;
    public static final int quietCntMod = 1, quietLineCntMod = 20, quietColCntMod = 25;
    
    protected static EventLocationFactory elFactory  = null;
    protected static BMEventSimulator eventSimulator = null;
    protected static BMQuerySimulator querySimulator = null;
	    
    protected static ELMDatabaseWriter dbWriter = null;
    protected static BMDatabaseReader  dbReader = null;
	    
    protected static RecognitionManager recManager = null;
    

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


    private static void runInsertions(int n, String configName)
	    throws java.sql.SQLException, 
		   elm.event.EventIDException,
		   java.io.IOException,
		   elm.event.WKTParseException,
		   elm.util.CircularBufferException {

	    // throw new RuntimeException("UNDER CONSTRUCTION!");
	    
	    long time_m = 0, time_e = 0, time_e2, time_m1, time_m2;
	    
	    Event e = null;
	    
// 	    recManager = new RecognitionManager();
// 	    recManager.registerRecognizer(new BMDummyRecognizer(dbWriter, 
// 								recManager, 
// 								eventSimulator, 
// 								querySimulator)); 
// 	    
// 	    recManager.start();
	    
	    if (verbose)
		System.out.print("Running " + n + " insertions with configuration " + configName + "...");
	    
	    int eventCnt;
	    for (eventCnt = 0; eventCnt < n; eventCnt++) {

		time_m1 = System.currentTimeMillis();
// 		e = eventSimulator.moveAndGenerateAtomicEvent();
		e = eventSimulator.generateEvent();
		time_m2 = System.currentTimeMillis();
	
		// System.out.println("about to store:\n" + e);
		dbWriter.storeEvent(e);
		querySimulator.newStoredEvent(e);
		// recManager.newEvent(e);
		
		time_e2 = System.currentTimeMillis();
		time_m += time_m2 - time_m1;
		time_e += time_e2 - time_m2;
	    }
/*
	    System.out.print("\nwaiting for recognizers to finish... ");
	    recManager.interrupt();
	    recManager.finishQueue();
	    System.out.println("done.");*/
	   
	    if (verbose) {		
		System.out.println("\ngenerated " + eventCnt + " events");
		System.out.println("time_m: " + time_m + "\ntime_e: " + time_e + 
				   " (not containing recognizers' store calls!)");
	    }


    }
    
    private static void runRandomQueries(int n, String configName, int resultSetLimit) 
	    throws java.sql.SQLException, 
		   elm.event.EventIDException,
		   java.io.IOException,
		   elm.event.WKTParseException,
		   elm.dbio.ELMDatabaseReaderException,
		   RuntimeException {

	    // throw new RuntimeException("UNDER CONSTRUCTION!");
	    
	    long time_m = 0, time_e = 0, time_e2, time_m1, time_m2;
	    
	    if (verbose)
		System.out.print("Running " + n + " queries with configuration " + configName + "...");
	    
	    int queryCnt;
	    for (queryCnt = 0; queryCnt < n; queryCnt++) {

		time_m1 = System.currentTimeMillis();
		BMQuery bmq = querySimulator.generateBMQuery();
		time_m2 = System.currentTimeMillis();

		java.util.Vector<Event> results = 
			dbReader.getMatchingEvents(bmq, resultSetLimit, 0, 
						   queryOrderMode);
		if (verbose)
			System.out.print("got back " + results.size() + " events");
		time_e2 = System.currentTimeMillis();
		time_m += time_m2 - time_m1;
		time_e += time_e2 - time_m2;
	    }
	   
	    if (verbose) {		
		System.out.println("\ngenerated " + queryCnt + " queries");
		System.out.println("time_m: " + time_m + "\ntime_e: " + time_e + 
				   " (not containing recognizers' store calls!)");
	    }
    }  
      
    private static void runQueriesInSets(int n, String configName, int resultSetLimit) 
	    throws java.sql.SQLException, 
		   elm.event.EventIDException,
		   java.io.IOException,
		   elm.event.WKTParseException,
		   elm.dbio.ELMDatabaseReaderException,
		   RuntimeException {

	    // throw new RuntimeException("UNDER CONSTRUCTION!");
	    
	    long time_m = 0, time_e = 0, time_e2, time_m1, time_m2;
	    
	    if (verbose)
		System.out.print("Running " + n + " queries with configuration " + configName + "...");
	    
	    int queryCnt;
	    for (queryCnt = 0; queryCnt < n; queryCnt++) {

		time_m1 = System.currentTimeMillis();
		BMQuery[] bmqa = querySimulator.generateBMQueryArray();
		// EventTemplate[] ta = querySimulator.generateEventTemplateArray();
		time_m2 = System.currentTimeMillis();
		
		for (BMQuery bmq : bmqa) {
		    
		    java.util.Vector<Event> results = 
			    dbReader.getMatchingEvents(bmq, resultSetLimit, 0, 
						       queryOrderMode);
		    if (verbose)
			System.out.println("got back " + results.size() + " events");
		    
		}
		time_e2 = System.currentTimeMillis();
		time_m += time_m2 - time_m1;
		time_e += time_e2 - time_m2;
	    }
	   
	    if (verbose) {
		    
		System.out.println("\ngenerated " + queryCnt + " queries");
		System.out.println("time_m: " + time_m + "\ntime_e: " + time_e + 
				" (not containing recognizers' store calls!)");
	    }
    }  
    
    
    protected static void printETH(Vector<String[]> eth) {
	
	for (String[] st: eth) {
	    System.out.println(st[0] + " " + st[1]);
	}
    }
    
    protected static void insertETH(Connection connection, Vector<String[]> eth) throws SQLException {
	
	System.out.print("inserting data into type hierarchy...");
	
	PreparedStatement ps = 
		connection.prepareStatement("INSERT INTO " + 
					    TableNames.event_type_hierarchy + "(" + 
					    ColumnNames.event_type + ", " +
					    ColumnNames.sub_type + ") VALUES (?, ?)");
	
	for (String[] st: eth) {
	    ps.setString(1, st[1]); 
	    ps.setString(2, st[0]); 
	    ps.executeUpdate();
	}
	
	System.out.print(" done.\n");
    }

    protected static void computeETHTransitiveHull(Connection connection) throws SQLException  {
	
	    EventTypeHierarchyIO ethIO = new EventTypeHierarchyIO(connection);
	    
	    System.out.print("reloading type hierarchy...");
	    TypeHierarchy<EventType> eth = ethIO.loadSourceTypeHierarchy();
	    // System.err.println("eth: " + eth);
	    System.out.print(" done.\ncomputing transitive closure and storing...");
	    ethIO.computeAndStoreTransitiveClosure(eth);
	    System.out.print(" done.\n");

    }
    
    public static void printUsage(boolean exit) {
	System.err.println("usage: RunELMBenchmark [--prepare]");
	System.err.println("or:    RunELMBenchmark <config-name>");
	if (exit)
	    System.exit(1);
    }

    public static void main(String[] args) {

	try {
	    // throw new RuntimeException("UNDER CONSTRUCTION!");
	    
	    String configName = "testconfigX1";
	    Connection connection = connect();
	    
	    if (args.length > 0) {
		if (args.length == 1) {
		    if (args[0].equals("--prepare")) {
			insertETH(connection, eventSimulator.generateTypeHierarchy());
			computeETHTransitiveHull(connection);
			System.exit(0);
		    }
		    else 
			configName = args[0];		    
		}
	    }
	    else
		printUsage(true);	    
	    
	    elFactory  = new EventLocationFactory();
	    eventSimulator = new BMEventSimulator(elFactory);
	    querySimulator = new BMQuerySimulator(elFactory);
	    
	    dbWriter = new ELMDatabaseWriter(connection);
	    dbReader = new BMDatabaseReader(connection, elFactory);
	    
	    System.out.println("Setting current BM config name to \"" + configName + "\"");
	    dbWriter.setCurrentBMConfigName(configName);	
	    
	    if (verbose)
		dbReader.setDebugMode(true);
	    	    
	    for (int currentRunNo = firstRunNo; currentRunNo <= lastRunNo; currentRunNo++) {
		
		System.out.println("Time: " + new java.util.Date());

		System.out.println("\n\nBlanking memory store...");
		dbWriter.flushMemoryStore();
		System.out.println("Sleeping " + sleepTime + " ms...");
		Thread.sleep(sleepTime);
		System.out.println("Starting run number " + currentRunNo + ": ");
		
		querySimulator.flushBuffer();
		dbWriter.resetEventCnt();
		dbWriter.turnOnDBBenchmark(configName, currentRunNo);	    
		dbReader.turnOnDBBenchmark(configName, currentRunNo, dbWriter);
		
		int quietLineCnt = 0, quietColCnt = 0;
		for (int i = 0; i < rounds; i++) {
		    
		    if (verbose)
			System.out.println("\n------------------------------------------\n");
		    
		    runInsertions(insertionsPerRound, configName);
		    
		    if (verbose)
			System.out.println("\n");
		
		    if (queryMode == 1)
			runQueriesInSets(querySetsPerRound, configName, queryLimit);
		    else if (queryMode == 2)
			runRandomQueries(queriesPerRound, configName, queryLimit);
	            else
			throw new Exception("unknown queryMode: " + queryMode);
		    
		    if (!verbose) {
			if (quietLineCnt % quietLineCntMod == 0)
			    System.out.println("");
			if (quietColCnt % quietColCntMod == 0) {
			    quietLineCnt++;
			    System.out.println("");			   
			}
			if (i % quietCntMod == 0) {
			    System.out.print(".");
			    quietColCnt++;
			}			
		    }
			
		}
		System.out.println("");
	    }
	    System.out.println("Time: " + new java.util.Date());

	}
	catch (Exception e) {
	    e.printStackTrace();
	}

    }

}
