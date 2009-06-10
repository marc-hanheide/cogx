package celmarchitecture.subarchitectures.elmwriter;

import java.util.Properties;
import java.util.concurrent.PriorityBlockingQueue;

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
import celm.conversion.*;
import celm.util.*;

import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;

import NavData.RobotPose;


/**
 *  LocationMonitor keeps track of the robot's position via RobotPose 
 *  updates and stores this information to fuse it with partial event
 *  information. <br>
 *  "RobotPose + CELM_PartialEventToStore = CELM_EventToStore"
 *  @author Dennis Stachowicz
 */
public class LocationMonitor extends PrivilegedManagedProcess {

    public static final double bufferDistance   = GlobalSettings.defaultAtomicBuffer;
    public static final int    posBufferSize    = 1024 * 1024;
    public static final int    pbqCapacity      = 1024;

    public static final String tooEarlyKey      = "LocationMonitorNOTE";
    public static final String tooEarlyValue    = "TOO_EARLY_NO_POSITION";
    
    public static final double[] loc0Coords     = new double[]{0, 0};
    public static final double   loc0Buffer     = 0.01;
    
    private TimedPositionBuffer posBuffer       = new TimedPositionBuffer(posBufferSize);


    private PriorityBlockingQueue<CELM_PartialEventToStore> pEventQueue   = 
	new PriorityBlockingQueue<CELM_PartialEventToStore>(pbqCapacity, new CastELMPartialEventComparator());

    private boolean localVerbose                = false; // true;
    private boolean verbose                     = GlobalSettings.verbose || localVerbose;
    
    private boolean strictMatch                 = false;
    
    private boolean inELMWriterSA               = false || GlobalSettings.singleSA;
    private SANames saNames                     = new SANames();
    
    
    // might need adjustment later on...
    private EventLocationFactory elf            = new EventLocationFactory();

    private EventConverter converter            = new EventConverter();
    
    
    public LocationMonitor(String _id) {

        super(_id);

    }
    
    public void configure(Properties config) {

	super.configure(config);

	saNames.configure(config);
    }
 
   
    @Override
    public void start() {
        super.start();

        try {
	    WorkingMemoryChangeReceiver wmcrRobotPose = new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {

			// log(CASTUtils.toString(_wmc));
			addRobotPose(_wmc);			
                    }
		};

	    WorkingMemoryChangeReceiver wmcrPartialEvent = new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {

			// log(CASTUtils.toString(_wmc));
			addPartialEvent(_wmc);
		    }
		};

	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(RobotPose.class, WorkingMemoryOperation.ADD), 
			    wmcrRobotPose);
	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(RobotPose.class, 
						   WorkingMemoryOperation.OVERWRITE), 
			    wmcrRobotPose);       

	    
	    addChangeFilter(ChangeFilterFactory.
			    createLocalTypeFilter(CELM_PartialEventToStore.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrPartialEvent);
               
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    System.exit(GlobalSettings.exitValueOnException);
        }

    }

   
    private void addRobotPose(WorkingMemoryChange _ceventChange) {
        
        try {
	    if (verbose)
		println("found RobotPose");

	    CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.m_address);
	    
            RobotPose pose = (RobotPose) wme.getData();
          	   
	    posBuffer.addPosition(new TimedPosition(pose));
	    	    
	    
	    if (verbose) 
		println("stored RobotPose");
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
    }    



    private void addPartialEvent(WorkingMemoryChange _ceventChange) {
        // get the action from wm
        try {
	    
	    CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.m_address);
	    
	    CELM_PartialEventToStore partialCEvent = 
		(CELM_PartialEventToStore) wme.getData();

	    if (verbose)
		println("found partial event of type \"" + 
			partialCEvent.m_eventType + "\" adding to queue...");
	    
	    pEventQueue.add(partialCEvent);

	    deleteFromWorkingMemory(_ceventChange.m_address);
		
	    if (verbose)
		println("deleted partial event from WM");
	    
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
    }    
    
    
    @Override
    public void runComponent() {
	
	try {
	    // prepare a default location for the case we need to record
	    // "early events"	    
	    EventLocation loc0 = elf.fromPoint(loc0Coords, loc0Buffer);
	    String loc0String = loc0.getWKTString();
	    
	    while (posBuffer.empty()) 
		Thread.sleep(10);

            while (m_status == ProcessStatus.RUN) {
                // do nothing for a while
                // Thread.sleep(1000);
		// Thread.sleep(5);

                // must check we're still running after sleep!
                if (m_status == ProcessStatus.RUN) {

		    CELM_PartialEventToStore pets = pEventQueue.take();

		    if (verbose)
			println("got:" + 
				"\nbegin:    " + pets.m_eventTime.m_begin.m_milliseconds + 
				"\nEARLIEST: " + posBuffer.getEarliestTimeValue() + 
				"\nend:      " + pets.m_eventTime.m_end.m_milliseconds +
				"\nLATEST:   " + posBuffer.getLatestTimeValue());
		    
		    if (pets.m_eventTime.m_begin.m_milliseconds < posBuffer.getEarliestTimeValue()) {
			// harsh reaction for the moment,
			// can be adjusted later
			if (posBuffer.full()) {
			    println("POSITION BUFFER SIZE TOO SMALL!!! HAVE TO DROP PARTIAL EVENT...");
			    
			    if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
			}
			else {
			    if (strictMatch) 
				println("PARTIAL EVENT TOO EARLY, no position " + 
					"information available. Sorry, have to DROP IT!");
			    else {
				// use a fixed default location, ...
				CELM_EventToStore ets = 
					converter.getEventToStore(pets, new CELM_EventLocation(loc0String));
				
				if (verbose)
				    println("got an early event of type " + ets.m_event.m_eventType +
					    ", saving it with a default location and a note");
				
				// ... add a note about it...
				EventSpecificFeatures esf =  converter.toEventSpecificFeatures(ets.m_event.m_eventSpecificFeatures);
				esf.addKeyValuePair(tooEarlyKey, tooEarlyValue);
				ets.m_event.m_eventSpecificFeatures = converter.toCEventSpecificFeatures(esf);
				
				// ... and finally save it
				if (inELMWriterSA)
				    addToWorkingMemory(newDataID(), ets);
				else
				    addToWorkingMemory(newDataID(), saNames.writerSA, ets);
			    }
			}
		    }
		    else if (pets.m_eventTime.m_end.m_milliseconds > posBuffer.getLatestTimeValue()) {
			// we do not know yet about positions at that time,
			// but we will later (hopefully)...
			if (verbose)
			    println("too early, waiting for more position information to come...");
			pEventQueue.add(pets);
			Thread.sleep(500);
		    }
		    else {
			// lock from external access // NEEDED???
			lockProcess(); 

			double[][] coordinatesArray = 
			    posBuffer.getPositionArray(pets.m_eventTime.m_begin.m_milliseconds,
						       pets.m_eventTime.m_end.m_milliseconds);

			EventLocation loc = elf.fromLinestring(coordinatesArray, bufferDistance);
			
			addToWorkingMemory(newDataID(), 
					   converter.getEventToStore(pets, 
								     new CELM_EventLocation(loc.getWKTString())));

			// let other stuff happen if necessary // NEEDED?
			unlockProcess();

			if (verbose)
			    println("added full event");
		    }
                  
                    
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
	/*
	catch (ConversionException e) {
            e.printStackTrace();
        }
	*/
	catch (PositionNotFoundException e) {
            e.printStackTrace();
	    
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }


	// FINALLY CLAUSE FOR unlockProcess???

    }
    




    @Override
    protected void taskAdopted(String _taskID) {}

    @Override
    protected void taskRejected(String _taskID) {}

    
    

}
