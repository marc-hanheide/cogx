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

import object.autogen.ObjectData.*;


import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;
import celm.conversion.BALTTimeConverter;
import celmarchitecture.global.GlobalSettings;


/** 
 *  ObjectMonitor is a simple monitor process for changes on Object SA's WM 
 *  which might be interesting (currently only FoundObject).
 *  @author Dennis Stachowicz
 */
public class ObjectMonitor extends SimpleAbstractWMMonitor {

 
    public ObjectMonitor(String _id) {
        super(_id);
    }

    
    @Override
    public void start() {
        super.start();

        try {
	    WorkingMemoryChangeReceiver wmcrProcessAddOverwriteEvent = 
		new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processAddOverwriteEvent(_wmc);			
                    }
		};
	       

	    addGlobalAddOverwriteFilter(FoundObject.class, wmcrProcessAddOverwriteEvent);

	    
	   
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }



    protected void processAddOverwriteEvent(WorkingMemoryChange _wmc) {
	
        try {
	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof FoundObject) {
		FoundObject fo = (FoundObject) data;
		Date time = BALTTimeConverter.toJavaDate(fo.m_time);
		EventSpecificFeatures esf = new EventSpecificFeatures(4);
	
		esf.addKeyValuePair("m_objectType", fo.m_objectType);
		esf.addKeyValuePair("m_object_unique", "" + fo.m_object_unique);
		esf.addKeyValuePair("m_object_x", "" + fo.m_object_x);
		esf.addKeyValuePair("m_object_y", "" + fo.m_object_y);
		esf.addKeyValuePair("m_object_z", "" + fo.m_object_z);
		esf.addKeyValuePair("m_robot_x", "" + fo.m_robot_x);
		esf.addKeyValuePair("m_robot_y", "" + fo.m_robot_y);

		addPartialEvent("FoundObject", 
				EventSpecificBinaryDataIO.objectToByteArray(fo), 
				time, 
				time, 
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