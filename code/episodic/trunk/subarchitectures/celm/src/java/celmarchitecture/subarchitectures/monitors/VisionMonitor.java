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

import visionarch.autogen.Vision.*;

import elm.event.EventSpecificBinaryDataIO;
import elm.event.EventSpecificFeatures;
import celm.conversion.BALTTimeConverter;
import celmarchitecture.global.GlobalSettings;



/** 
 *  VisionMonitor is a simple monitor process for processing results from Vision 
 *  which might be interesting (currently only SceneObject and ImageFrame structs).
 *  @author Dennis Stachowicz
 */
public class VisionMonitor extends SimpleAbstractWMMonitor {


    public VisionMonitor(String _id) {
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
	    WorkingMemoryChangeReceiver wmcrProcessDeleteEvent = 
		new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			// log(CASTUtils.toString(_wmc));
			processDeleteEvent(_wmc);			
                    }
		};

	   
	    addGlobalAddOverwriteFilter(SceneObject.class, wmcrProcessAddOverwriteEvent);
	    addGlobalDeleteFilter(SceneObject.class, wmcrProcessDeleteEvent);

	    addGlobalAddOverwriteFilter(ImageFrame.class, wmcrProcessAddOverwriteEvent);

	   
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }

 
    protected void processDeleteEvent(WorkingMemoryChange _wmc) {


        try {
	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof SceneObject) {
		
		EventSpecificFeatures esf = new EventSpecificFeatures(1);
		

		// since Vision (see AttendedObjectList) might refer to 
		// addresses we just save them, too, here
		esf.addKeyValuePair("wm address id", _wmc.m_address.m_id);
			
		addPartialEvent("SceneObject deleted", 
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
 
    
    protected void processAddOverwriteEvent(WorkingMemoryChange _wmc) {


        try {
	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof ImageFrame) {
		ImageFrame iframe = (ImageFrame) data;
		Date time = BALTTimeConverter.toJavaDate(iframe.m_time);
		EventSpecificFeatures esf = new EventSpecificFeatures(4);
	
		esf.addKeyValuePair("m_width", "" + iframe.m_width);
		esf.addKeyValuePair("m_height", "" + iframe.m_height);
		// -> eventSpecificBinaryData?
		// esf.addKeyValuePair("m_image", "" + iframe.m_image);
		esf.addKeyValuePair("m_camNum", "" + iframe.m_camNum);		
		addPartialEvent("ImageFrame", 
				EventSpecificBinaryDataIO.objectToByteArray(iframe), 
				time, 
				time, 
				null, 
				esf);
	    }
	    else if (data instanceof SceneObject) {
		SceneObject sceneObject = (SceneObject) data;
		Date time = BALTTimeConverter.toJavaDate(sceneObject.m_time);
		EventSpecificFeatures esf = new EventSpecificFeatures(6);
		
		esf.addKeyValuePair("m_label", "" + sceneObject.m_label);
		esf.addKeyValuePair("m_color value", 
				    "" + sceneObject.m_color.m_int);
		esf.addKeyValuePair("m_color confidence", 
				    "" + sceneObject.m_color.m_confidence);
		esf.addKeyValuePair("m_shape value", 
				    "" + sceneObject.m_shape.m_int);
		esf.addKeyValuePair("m_shape confidence", 
				    "" + sceneObject.m_shape.m_confidence);

		esf.addKeyValuePair("m_pose position x", "" + sceneObject.m_pose.m_position.m_x);
		esf.addKeyValuePair("m_pose position y", "" + sceneObject.m_pose.m_position.m_y);
		esf.addKeyValuePair("m_pose position z", "" + sceneObject.m_pose.m_position.m_z);
		
		esf.addKeyValuePair("m_pose orientation x", "" + sceneObject.m_pose.m_orientation.m_x);
		esf.addKeyValuePair("m_pose orientation y", "" + sceneObject.m_pose.m_orientation.m_y);
		esf.addKeyValuePair("m_pose orientation z", "" + sceneObject.m_pose.m_orientation.m_z);
	
		// Currently not saved: BBox
		// esf.addKeyValuePair("m_bbox ...
		

		// since Vision (see AttendedObjectList) might refer to 
		// addresses we just save them, too, here
		esf.addKeyValuePair("wm address id", _wmc.m_address.m_id);

	
		addPartialEvent("SceneObject", 
				EventSpecificBinaryDataIO.objectToByteArray(sceneObject), 
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