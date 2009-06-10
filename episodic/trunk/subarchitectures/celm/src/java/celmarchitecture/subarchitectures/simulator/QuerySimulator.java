package celmarchitecture.subarchitectures.simulator;

import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

import cast.core.data.CASTData;

import elm.event.*;

import celm.autogen.*;
import celm.conversion.EventConverter;
import celm.conversion.ConversionException;

import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;


/** 
 *  QuerySimulator is a process which can be used to quickly test
 *  whether recollection for recent events works. It repeatedly
 *  asks which events happened in the last 2 seconds.
 *
 *  @author Dennis Stachowicz
 */
public class QuerySimulator extends PrivilegedManagedProcess {

    public static final int qLimit        = 10;


    private boolean localVerbose          = false; // true;
    private boolean verbose               = GlobalSettings.verbose || localVerbose;


    private EventConverter converter      = new EventConverter();

    private boolean singleSA              = true; // false;    
    private SANames saNames               = new SANames();

    
    public QuerySimulator(String _id) {
        super(_id);
    }

    public void configure(Properties config) {
	super.configure(config);

	saNames.configure(config);
    }

 
    /*
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        super.start();

    }

    
    private void processRecollectionResults(WorkingMemoryChange _ceventChange) {
        
        try {

	    CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.m_address);
	    

	    CELM_EventQuery query = (CELM_EventQuery) wme.getData();
	    
// 	    for (int i = 0; i < query.m_events.length; i++)
// 		println(i + ": " + converter.getEvent(query.m_events[i]));
	    
	    deleteFromWorkingMemory(_ceventChange.m_address);
		
	    if (verbose)
		println("deleted query (and results) from working memory");
 
	    
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
// 	catch (WKTParseException e) {
//             e.printStackTrace();
//         }
	
// 	catch (ConversionException e) {
//             e.printStackTrace();
//         }

    }    


    protected void generateQuery() 
	throws ConversionException, SubarchitectureProcessException {


	if (verbose)
	    println("generating a new query...");

	EventTemplate template = new EventTemplate();

	// what happened in (or at least overlapping) the last 2 seconds?
	long currentTime = System.currentTimeMillis();
	template.time = new EventTime(currentTime - 2000, currentTime);
	template.timeMatchMode = template.matchIntersectionNotEmpty;


	String dataID = newDataID();
	if (singleSA) {
	    addChangeFilter(ChangeFilterFactory.createIDFilter(dataID, 
							       WorkingMemoryOperation.OVERWRITE),
			    new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				    processRecollectionResults(_wmc);
				}
			    });
       
	    addToWorkingMemory(dataID, 
			       new CELM_EventQuery(qLimit, 
						   converter.getEventCue(template),
						   new CELM_StoredEvent[0]));
	}
	else {
	    addChangeFilter(ChangeFilterFactory.createAddressFilter(dataID, 
								    saNames.recollectionSA,
								    WorkingMemoryOperation.OVERWRITE),
			    new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				    processRecollectionResults(_wmc);
				}
			    });
	    
	    addToWorkingMemory(dataID, 
			       saNames.recollectionSA,
			       new CELM_EventQuery(qLimit, 
						   converter.getEventCue(template),
						   new CELM_StoredEvent[0]));

	    
	}


	if (verbose)
	    println("new query (and change filter!) added.");


    }


    /*
     * @see cast.core.components.CASTComponent#runComponent()
     */
    @Override
    public void runComponent() {
	
        try {
            while (m_status == ProcessStatus.RUN) {
                // do nothing for a while
                Thread.sleep(1000);
		// Thread.sleep(5);

                // must check we're still running after sleep!
                if (m_status == ProcessStatus.RUN) {

                    // lock from external access // NEEDED???
                    lockProcess();

		    generateQuery();
	

                    // let other stuff happen if necessary
                    unlockProcess();
                }
            }
        }
        catch (InterruptedException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	catch (ConversionException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }


	// FINALLY CLAUSE FOR unlockProcess???

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
