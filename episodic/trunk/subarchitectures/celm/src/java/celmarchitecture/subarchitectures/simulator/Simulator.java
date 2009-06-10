package celmarchitecture.subarchitectures.simulator;


import java.util.Properties;

import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;

import java.io.IOException;

import elm.event.Event;
import elm.event.EventIDException;
import elm.tests.EventSimulator;

import celm.autogen.*;
import celm.conversion.EventConverter;

import celmarchitecture.global.GlobalSettings;
import celmarchitecture.global.SANames;


import NavData.RobotPose;


/** 
 *  Simulator can generate fake events of different kinds: full events, partial events
 *  and RobotPoses. Useful for testing and development, not necessary in an integrated system.
 *
 *  @author Dennis Stachowicz
 */
public class Simulator extends PrivilegedManagedProcess {
    
    public static final boolean storeEventsLocally  = GlobalSettings.singleSA;
    
    public static final boolean createFullEvents    = true;
    public static final boolean createPartialEvents = true;
    public static final boolean createRobotPoses    = true;    
    
    public static final int     robotPosesCnt       = 10;


    private boolean localVerbose                    = false;
    private boolean verbose                         = GlobalSettings.verbose || localVerbose;
    private SANames saNames                         = new SANames();

    

    private EventSimulator privSimulator            = new EventSimulator();
    private EventConverter converter                = new EventConverter();

   
    public Simulator(String _id) {
        super(_id);
    }
    
    public void configure(Properties config) {
	super.configure(config);

	saNames.configure(config);
    }


    private CELM_EventToStore generateEventToStore() {

	CELM_EventToStore ce = null;

	try {

	    Event e = privSimulator.moveAndGetAtomicEvent();

	    
	    if (verbose) {
		println("generated EventToStore\n");
		balt.corba.autogen.FrameworkBasics.BALTTime bt = 
			balt.management.ProcessLauncher.getBALTTime();
		println("BALTTime: " + bt.m_s + ", " + bt.m_us +
			"\nmatches Java Date: " + celm.conversion.BALTTimeConverter.toJavaDate(bt));
		
	    }

	    ce = converter.getEventToStore(e);
	   
	}
	catch (EventIDException e) {
            e.printStackTrace();
        }
	catch (IOException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
	return ce;
    }


    private CELM_PartialEventToStore generatePartialEventToStore() {

	CELM_EventTime etime = 
	    new CELM_EventTime(new CELM_EventTimestamp(System.currentTimeMillis() - 2000),
			       new CELM_EventTimestamp(System.currentTimeMillis()));


	CELM_PartialEventToStore ce = 
	    new CELM_PartialEventToStore("partial test type 1",
					 etime,
					 new String[0],
					 new CELM_EventSpecificFeaturesEntry[0],
					 new byte[0]);


	return ce;
    }

    private RobotPose generateRobotPose() {

	RobotPose rp = new RobotPose();

	rp.m_time = balt.management.ProcessLauncher.getBALTTime();

	privSimulator.move();
	double[] pos = privSimulator.getPosition();

	rp.m_x = pos[0];
	rp.m_y = pos[1];
	rp.m_theta = 0;

	rp.m_cov = new double[9];
	for (int i = 0; i < rp.m_cov.length; i++)
	    rp.m_cov[i] = 0;
	    
	return rp;
    }


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

		    if (verbose)
			println("simulating...");

		    if (createFullEvents)
			if (storeEventsLocally)
			    addToWorkingMemory(newDataID(), generateEventToStore());
			else
			    addToWorkingMemory(newDataID(), saNames.writerSA, generateEventToStore());
			
		    if (createRobotPoses)
			for (int i = 0; i < robotPosesCnt; i++)
			    if (storeEventsLocally)
				addToWorkingMemory(newDataID(), generateRobotPose()); 
			    else
				addToWorkingMemory(newDataID(), saNames.writerSA, generateRobotPose());
			    
		    if (createPartialEvents)
			if (storeEventsLocally)
			    addToWorkingMemory(newDataID(), generatePartialEventToStore());
			else
			    addToWorkingMemory(newDataID(), saNames.writerSA, generatePartialEventToStore());
	                    
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

	// FINALLY CLAUSE FOR unlockProcess???

    }

    
    
  
    @Override
	protected void taskAdopted(String _taskID) {}

    @Override
	protected void taskRejected(String _taskID) {}
        
    
}
