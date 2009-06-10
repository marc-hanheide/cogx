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

import ComaData.*;

import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;
import celm.conversion.BALTTimeConverter;
import celmarchitecture.global.EventTypeNames;
import celmarchitecture.global.GlobalSettings;


/** 
 *  ComaMonitor is a simple monitor process for changes on Coma WM 
 *  which might be interesting.
 *  @author Dennis Stachowicz
 */
public class ComaMonitor extends SimpleAbstractWMMonitor {

  
    public ComaMonitor(String _id) {
        super(_id);
    }


    
    @Override
    public void start() {
        super.start();

        try {
	    WorkingMemoryChangeReceiver wmcrProcessAddEvent = new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processEvent(_wmc, "ADD");			
                    }
		};
		
	    WorkingMemoryChangeReceiver wmcrProcessOverwriteEvent = new WorkingMemoryChangeReceiver() {

		    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processEvent(_wmc, "OVERWRITE");			
		    }
		};
	    
	    WorkingMemoryChangeReceiver wmcrProcessDeleteEvent = new WorkingMemoryChangeReceiver() {

		    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processEvent(_wmc, "DELETE");			
		    }
		};
    

	    addGlobalAddFilter(ComaInstance.class, wmcrProcessAddEvent);
	    addGlobalOverwriteFilter(ComaInstance.class, wmcrProcessOverwriteEvent);
	    addGlobalDeleteFilter(ComaInstance.class, wmcrProcessDeleteEvent);
	   
               
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
	    
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }


    protected void processEvent(WorkingMemoryChange _wmc, String wmOperation) {

        try {
	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof ComaInstance) {

		ComaInstance ci = (ComaInstance) data;
		
		EventSpecificFeatures esf = new EventSpecificFeatures(7);
		
		esf.addKeyValuePair("wm address id", _wmc.m_address.m_id);
		esf.addKeyValuePair("wm operation", wmOperation);
		
		esf.addKeyValuePair("m_namespace", ci.m_namespace);
		esf.addKeyValuePair("m_sep", ci.m_sep);
		esf.addKeyValuePair("m_name", ci.m_name);
		esf.put("m_mostSpecificConcepts", ci.m_mostSpecificConcepts);
		esf.put("m_givenNames", ci.m_givenNames);
				
		addPartialEvent("ComaInstance", 
				EventSpecificBinaryDataIO.objectToByteArray(ci), 
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
    
}