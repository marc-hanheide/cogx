package celmarchitecture.subarchitectures.elmwriter;

import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.OperationMode;
// import cast.cdl.testing.CAST_TEST_FAIL;
// import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.data.CASTData;


import java.sql.Connection;
import java.sql.DriverManager;
import java.sql.SQLException;

import elm.event.AtomicEvent;
import elm.event.EventTime;
import elm.event.EventType;
import elm.event.EventLocationFactory;

import elm.event.Event;
import elm.event.EventIDException;
import elm.event.WKTParseException;
import elm.dbio.ELMDatabaseWriter;

import celm.autogen.*;
import celm.conversion.EventConverter;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;
import celmarchitecture.global.DBConfig;




/** 
 *  ElmWriter provides an interface from CAST to the ELM memory store. 
 *  @author Dennis Stachowicz
 */
public class ElmWriter extends PrivilegedManagedProcess {


    private boolean localVerbose          = false;
    private boolean verbose               = GlobalSettings.verbose || localVerbose;

    private boolean singleSA              = GlobalSettings.singleSA;
    
    private DBConfig dbConfig             = new DBConfig();
    private SANames  saNames              = new SANames();
    
    private EventConverter converter      = new EventConverter();
    private ELMDatabaseWriter dbWriter    = null;
    
    public ElmWriter(String _id) {
        super(_id);       
    }

    public void configure(Properties config) {
	super.configure(config);

	dbConfig.configure(config);
	saNames.configure(config);
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
	    //The second and third arguments are the username and password,
	    //respectively.They should be whatever is necessary to connect
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
	    dbWriter = new ELMDatabaseWriter(connect("jdbc:postgresql://" + 
						     dbConfig.server + "/" + dbConfig.name,
						     dbConfig.user, dbConfig.passwd));
	    
            addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(CELM_EventToStore.class, WorkingMemoryOperation.ADD),
	    // addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(CELM_EventToStore.class, WorkingMemoryOperation.ADD),    
			     new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {

			storeEvent(_wmc);
                    }
                });

	    addWakeupEvent();
	    if (verbose)
		println("ELMWriter waking up...");
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	catch(SQLException se) {
	    println("Could not initialize ELMDatabaseWriter.");
	    se.printStackTrace();
	    
	    System.exit(GlobalSettings.exitValueOnException);
	}
    }

    private void addWakeupEvent() {
	
	long time = System.currentTimeMillis();
	double[] pos0 = { 0.0, 0.0};
	AtomicEvent event = 
	    new AtomicEvent(new EventType("--ElmWriter wakes up--"),
			    new EventTime(time, time),
			    new EventLocationFactory().fromPoint(pos0, 0.0001),
			    (byte[]) null,
			    null,
			    null);
	try {
	    dbWriter.storeEvent(event);		    
	}
        catch (Exception e) {
            e.printStackTrace();
	    
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	    

    }


    private void storeEvent(WorkingMemoryChange _ceventChange) {
        
        try {
	    if (verbose)
		println("going to store event...");

            CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.m_address);
            CELM_EventToStore cevent = (CELM_EventToStore) wme.getData();
            
	    Event event =  converter.getEvent(cevent);
	    dbWriter.storeEvent(event);

	    if (verbose)
		println("stored an event");
	    
	    if (singleSA)
		addToWorkingMemory(newDataID(), 
				   converter.getStoredEvent(event));
	    else 
		addToWorkingMemory(newDataID(), 
				   saNames.recognitionSA, 
				   converter.getStoredEvent(event));
	  
	  
            deleteFromWorkingMemory(_ceventChange.m_address.m_id);

	    if (verbose)
		println("deleted an event");

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
	catch (EventIDException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	catch (WKTParseException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }    

    
    @Override
    protected void taskAdopted(String _taskID) {}


    @Override
    protected void taskRejected(String _taskID) {}

}
