package celmarchitecture.subarchitectures.monitors;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;


import java.util.Date;

import planning.autogen.*;
import cast.cdl.TriBool;

import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;
import celm.conversion.BALTTimeConverter;
import celmarchitecture.global.GlobalSettings;



/** 
 *  PlanningMonitor is a simple monitor process for changes on Planning WM 
 *  which might be interesting (currently only PlanningProcessRequest structs).
 *  @author Dennis Stachowicz
 */
public class PlanningMonitor extends SimpleAbstractWMMonitor {

 
    /**
     * @param _id
     */
    public PlanningMonitor(String _id) {
        super(_id);
    }

    
    @Override
    public void start() {
        super.start();

        try {
	    WorkingMemoryChangeReceiver wmcrProcessAddEvent = 
		new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processAddEvent(_wmc);			
                    }
		};
	    WorkingMemoryChangeReceiver wmcrProcessOverwriteEvent = 
		new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processOverwriteEvent(_wmc);			
                    }
		};

	    /*
	    WorkingMemoryChangeReceiver wmcrProcessDeleteEvent = 
		new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processDeleteEvent(_wmc);			
                    }
		};
	    */


	    
	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(PlanningProcessRequest.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrProcessAddEvent);
	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(PlanningProcessRequest.class, 
						   WorkingMemoryOperation.OVERWRITE), 
			    wmcrProcessOverwriteEvent);       
	    
	    
               
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }


   
    protected void processAddEvent(WorkingMemoryChange _wmc) {

	
        try {
	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof PlanningProcessRequest) {
		PlanningProcessRequest ppr = (PlanningProcessRequest) data;

		EventSpecificFeatures esf = new EventSpecificFeatures(7);
		esf.addKeyValuePair("wm address id", _wmc.m_address.m_id);
		esf.addKeyValuePair("m_maplGoal", "" + ppr.m_maplGoal);
		esf.addKeyValuePair("m_cause m_id", "" + ppr.m_cause.m_address.m_id);
		esf.addKeyValuePair("m_cause m_subarchitecture", 
				    "" + ppr.m_cause.m_address.m_subarchitecture);
		esf.addKeyValuePair("m_execute", "" + ppr.m_execute);
		esf.addKeyValuePair("m_status", "" + ppr.m_status.value());
		esf.addKeyValuePair("m_succeeded", 
				    (ppr.m_succeeded.value() == TriBool._triTrue ? "true" : 
				     (ppr.m_succeeded.value() == TriBool._triFalse ? "false" : 
				      "indeterminate")));


		addPartialEvent("PlanningProcessRequest", 
				EventSpecificBinaryDataIO.objectToByteArray(ppr), 
				null, 
				null, 
				null, 
				esf);
	    }

        }
	catch (java.io.IOException e)  {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
    }


    protected void processOverwriteEvent(WorkingMemoryChange _wmc) {

	try {
	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof PlanningProcessRequest) {
		PlanningProcessRequest ppr = (PlanningProcessRequest) data;
		
		EventSpecificFeatures esf = new EventSpecificFeatures(7);
		esf.addKeyValuePair("wm address id", _wmc.m_address.m_id);
		esf.addKeyValuePair("m_maplGoal", "" + ppr.m_maplGoal);
		esf.addKeyValuePair("m_cause m_id", "" + ppr.m_cause.m_address.m_id);
		esf.addKeyValuePair("m_cause m_subarchitecture", 
				    "" + ppr.m_cause.m_address.m_subarchitecture);
		esf.addKeyValuePair("m_execute", "" + ppr.m_execute);
		esf.addKeyValuePair("m_status", "" + ppr.m_status.value());
		esf.addKeyValuePair("m_succeeded", 
				    (ppr.m_succeeded.value() == TriBool._triTrue ? "true" : 
				     (ppr.m_succeeded.value() == TriBool._triFalse ? "false" : 
				      "indeterminate")));
	
		addPartialEvent("PlanningProcessRequest", 
				null, 
				null, 
				null, 
				null, 
				esf);
	    }

        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
    }
    
}