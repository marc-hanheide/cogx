package celmarchitecture.subarchitectures.elmwriter;

import java.util.Properties;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

import cast.core.data.CASTData;

import celm.autogen.CELM_EventToStore;
import celm.autogen.CELM_PartialEventToStore;


import celmarchitecture.global.GlobalSettings;



/**
 *  A simple process which just moves CELM_EventToStore and 
 *  CELM_PartialEventToStore objects from other WMs to that of ElmWriter.
 *  That way processes in other SAs can also just report events on their own WM.
 *  @author ds
 */
public class EventMonitor extends PrivilegedManagedProcess {

    private boolean localVerbose                    = false;
    private boolean verbose                         = GlobalSettings.verbose || localVerbose;

   
    public EventMonitor(String _id) {

        super(_id);
    }

   
    @Override
    public void start() {
        super.start();

        try {
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(CELM_PartialEventToStore.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                       
			if (! _wmc.m_address.m_subarchitecture.equals(m_subarchitectureID))
			    movePartialEvent(_wmc);
			else
			    if (verbose)
				println("found partial event not to move");
                    }
                });

	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(CELM_EventToStore.class, WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        
			if (! _wmc.m_address.m_subarchitecture.equals(m_subarchitectureID))
			    moveEvent(_wmc);
			else
			    if (verbose)
				println("found event not to move");
                    }
                });
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }

 
    private void movePartialEvent(WorkingMemoryChange _ceventChange) {
        
        try {
	    if (verbose)
		println("found partial event to move");
	
	    CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.m_address);
	    
            CELM_PartialEventToStore cevent = (CELM_PartialEventToStore) wme.getData();
            
	    if (verbose)
		println("about to move");

	    addToWorkingMemory(newDataID(), cevent);
		
	    deleteFromWorkingMemory(_ceventChange.m_address);
		
	    if (verbose)
		println("moved a partial event");
	    
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
    }    

    private void moveEvent(WorkingMemoryChange _ceventChange) {
  
        try {
	    if (verbose)
		println("found event to move");


	    CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.m_address);
	    
	    CELM_EventToStore cevent = (CELM_EventToStore) wme.getData();
            
	    
	    if (verbose)
		println("about to move");

	    addToWorkingMemory(newDataID(), cevent);
		

	    deleteFromWorkingMemory(_ceventChange.m_address);
		
	    if (verbose)
		println("moved an event");
	    
        }
        catch (SubarchitectureProcessException e) {
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
