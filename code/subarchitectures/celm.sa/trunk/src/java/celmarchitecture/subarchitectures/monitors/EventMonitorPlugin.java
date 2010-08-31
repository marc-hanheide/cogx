package celmarchitecture.subarchitectures.monitors;

import java.util.Date;
import java.util.Vector;
import java.util.Map;
import java.util.HashMap;

import cast.cdl.WorkingMemoryOperation;

import elm.event.EventSpecificFeatures;
import celmarchitecture.global.GlobalSettings;


/**
 * EventMonitorPlugin is an interface defining the prototypes of methods
 * which can deduce event information (sensu ELM) from CAST WM changes.
 * See EventMonitorDefaultPlugin for a default implementation. 
 * The EventMonitorDefaultPlugin, however, can only extract some rough
 * event type-general information. You are encouraged to provide more 
 * specific information than this in your plugins.
 *
 * @see PBMonitor
 * @see EventMonitorDefaultPlugin
 *
 * @author Dennis Stachowicz
 */
public interface EventMonitorPlugin {

	/**
	  *  Return the event type indicated by a WM change involving the given parameter iceObject.
	  */
	public String getEventType(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject);

	/**
	  *  If you can deduct the start time of the event from the given parameter iceObject
	  *  you should do so. If you cannot return null and it will be estimated via the time
	  *  of the registered change event.
	  */
	public Date getStartTime(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject);

	/**
	  *  If you can deduct the end time of the event from the given parameter iceObject
	  *  you should do so. If you cannot return null and it will be estimated via the time
	  *  of the registered change event.
	  */
	public Date getEndTime(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject);

	/**
	  * (optional) If there is a binary payload (modal information) 
	  *  which should be stored with the event it can be specified by overriding this definition.
 	  *  Or just return null.
	  */		
	public byte[] getEventSpecificBinaryData(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject);


	/**
	  * (optional)  For other useful event information in string format, 
 	  *  more precisely, as a set of (name, value) pairs of strings.
	  *  If this is undesired, redefine as "return null" to leave
	  *  out this kind of additional information.
	  */
	public EventSpecificFeatures getEventSpecificFeatures(WorkingMemoryOperation wmOperation, Ice.ObjectImpl iceObject);

}



