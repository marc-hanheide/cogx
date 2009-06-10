package celmarchitecture.subarchitectures.recollection;

import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.OperationMode;
import cast.cdl.testing.CAST_TEST_FAIL;
import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.data.CASTData;

import java.util.Vector;

import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;

import elm.event.*;
import elm.dbio.ELMDatabaseReader;
import elm.dbio.ELMDatabaseReaderException;

import celm.autogen.*;
import celm.conversion.EventConverter;
import celm.conversion.ConversionException;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.DBConfig;




/** 
 *  Recollector is a process which matches events from the ELM 
 *  memory store with a cue it finds on its working memory.
 *  <br>
 *  Interaction with this process works as follows: Another process
 *  writes a CELM_EventQuery struct to Recollector's WM *after* 
 *  registering a WM change receiver on this particular item. 
 *  Recollector overwrites the struct, filling the m_events field. 
 *  Now the calling process can read results and delete the struct.
 *
 *  @see    CELM_EventQuery
 *  @see    CELM_EventCue
 *  @author Dennis Stachowicz
 */
public class Recollector extends PrivilegedManagedProcess {


    public static final int defaultLimit   = 50;

    private boolean localVerbose           = false; // true;
    private boolean verbose                = GlobalSettings.verbose || localVerbose;
    
    private EventConverter converter       = new EventConverter();
    private ELMDatabaseReader dbReader     = null;

    private DBConfig dbConfig              = new DBConfig();

    private EventLocationFactory elFactory = new EventLocationFactory();
   
   
 
    public Recollector(String _id) {
        super(_id);     
    }

    public void configure(Properties config) {
	super.configure(config);

	dbConfig.configure(config);
    }

    
    private Connection connect(String url, String user, String password) {
	Connection connection = null;

	try {
	    Class.forName("org.postgresql.Driver");
	}
	catch(ClassNotFoundException cnfe) {
	    println("Could not find the driver!");
	    cnfe.printStackTrace();
	    System.exit(1);
	}

	if (verbose)
	    println("Registered the driver.\nConnecting to the database... ");

	try {
	    // The second and third arguments are the username and password,
	    // respectively.They should be whatever is necessary to connect
	    // to the database.
	    connection = DriverManager.getConnection(url, user, password);
	}
	catch(SQLException se) {
	    println("Could not connect.");
	    se.printStackTrace();
	    System.exit(1);
	}
              
	if (verbose)
	    println("Connected.");
                
	return connection;
    }

 
    @Override
    public void start() {
        super.start();

        try {
	    dbReader = new ELMDatabaseReader(connect("jdbc:postgresql://" + 
						     dbConfig.server + "/" + dbConfig.name,
						     dbConfig.user, dbConfig.passwd),
					     elFactory);
	    dbReader.setDebugMode(verbose);
	    
            addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(CELM_EventQuery.class, 
								      WorkingMemoryOperation.ADD),
			    new WorkingMemoryChangeReceiver() {

				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				    
				    answerQuery(_wmc);
				}
			    });
    
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    System.exit(GlobalSettings.exitValueOnException);
        }
	catch(SQLException se) {
	    println("Could not initialize ELMDatabaseReader.");
	    se.printStackTrace();
	    System.exit(GlobalSettings.exitValueOnException);
	}
    }

 
    private void answerQuery(WorkingMemoryChange wmChange) {
        
        try {
	    if (verbose)
		println("found a new query...");

            CASTData<?> wme = getWorkingMemoryEntry(wmChange.m_address);
            CELM_EventQuery query = (CELM_EventQuery) wme.getData();
            
	    EventTemplate template   = converter.getEventTemplate(query.m_cue);

	    int limit = query.m_limit < 0 ? defaultLimit : query.m_limit;

	    Vector<Event> results = dbReader.getMatchingEvents(template, limit, 0);
	    
	    CELM_StoredEvent[] storedEvents = new CELM_StoredEvent[results.size()];  
	    for (int i = 0; i < results.size(); i++)
		storedEvents[i] = converter.getStoredEvent(results.get(i));


	    query.m_events = storedEvents;
	    overwriteWorkingMemory(wmChange.m_address, query);

	    if (verbose)
		println("results written to WM");

        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	catch (SQLException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	catch (ELMDatabaseReaderException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	catch (WKTParseException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	catch (ConversionException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }


    }    

    
    /*
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _taskID) {}

    /*
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _taskID) {}

}
