package celmarchitecture.subarchitectures.recognition;

import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
// import cast.cdl.testing.CAST_TEST_FAIL;
// import cast.cdl.testing.CAST_TEST_PASS;
import cast.core.data.CASTData;

import elm.event.Event;
import elm.event.WKTParseException;
import elm.eventrecognition.*;

import celm.autogen.*;
import celm.conversion.EventConverter;
import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;


/**
 *  The Recognizer process is notified of events arrived in the 
 *  ELM memory store and passes them over to a RecognitionManager
 *  (where specific handlers can be registered) for further processing.
 *  When one of the more specific handlers / recognizers find that
 *  some events together form a complex event this is reported back to 
 *  the C-ELM system (to ElmWriter, in particular) starting a new 
 *  cycle of processing...
 *
 *  @see    RecognitionManager
 *  @author Dennis Stachowicz
 */
public class Recognizer extends PrivilegedManagedProcess {


    private boolean localVerbose          = false;
    private boolean verbose               = GlobalSettings.verbose || localVerbose;
  
    private boolean singleSA              = GlobalSettings.singleSA;
    private SANames saNames               = new SANames();

    
    private EventConverter converter      = new EventConverter();
    private RecognitionManager recManager = null;

    
    public Recognizer(String _id) {
        super(_id);
       
	recManager = new RecognitionManager();
	recManager.registerRecognizer(new DummyRecognizerC(this));
	recManager.registerRecognizer(new BeWelcomedRecognizer(this));
    }

    public void configure(Properties config) {
	super.configure(config);

	saNames.configure(config);
    }


    protected void newEventDetected(Event event) {
	if (verbose)
	    println("new event detected: " + event);

	// store or return immediately to elmwriter WM
	try {
	    if (singleSA)
		addToWorkingMemory(newDataID(), 
				   converter.getEventToStore(event));
	    else
		addToWorkingMemory(newDataID(), 
				   saNames.writerSA,
				   converter.getEventToStore(event));
	}
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
    }

 
    /*
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        super.start();

        try {
	    recManager.start();

	    
            addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(CELM_StoredEvent.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processEvent(_wmc);
			
                    }
                });
	    
	   
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }

  
    private void processEvent(WorkingMemoryChange _ceventChange) {
       
        try {
	    if (verbose)
		println("found event to process");


	    CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.m_address);
	    
	    
            CELM_StoredEvent cevent = (CELM_StoredEvent) wme.getData();
            
	    if (verbose)
		println("adding event to queue");
	    recManager.newEvent(converter.getEvent(cevent));
	   
	    deleteFromWorkingMemory(_ceventChange.m_address);

	    if (verbose)
		println("done");
        }
        catch (SubarchitectureProcessException e) {
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
