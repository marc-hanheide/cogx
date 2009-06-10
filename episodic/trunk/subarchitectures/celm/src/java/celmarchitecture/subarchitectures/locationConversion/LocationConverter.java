package celmarchitecture.subarchitectures.locationConversion;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;


import java.util.Date;
import java.util.Vector;

import NavData.*;

import celmarchitecture.global.GlobalSettings;
import celm.autogen.CELM_EventLocation;
import locationConversion.autogen.*;

import elm.event.EventLocation;
import elm.event.EventLocationFactory;


/** 
 *  Converts to the location format understood by ELM/C-ELM. <br>
 *  WARNING: experimental, largely untested code!
 *  @author Dennis Stachowicz
 */
public class LocationConverter extends PrivilegedManagedProcess {

    public static final double     fNodeBuffer    = 1.0;

    // set this to GlobalSettings.defaultAtomicBuffer???
    public static final double     gatewayBuffer  = 0.1; 

    // more specific initialisation?
    protected EventLocationFactory elFactory      = new EventLocationFactory();
    protected RobotPose            lastPose       = null;
    protected NavGraph             lastNavGraph   = null;



    public LocationConverter(String _id) {
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
			    createGlobalTypeFilter(RobotPose.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrProcessAddOverwriteEvent);
	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(RobotPose.class, 
						   WorkingMemoryOperation.OVERWRITE), 
			    wmcrProcessAddOverwriteEvent);

	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(NavGraph.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrProcessAddOverwriteEvent);

	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(NavGraph.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrProcessAddOverwriteEvent);

	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(ConvertHere.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrProcessAddOverwriteEvent);

	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(ConvertPosition.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrProcessAddOverwriteEvent);

	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(ConvertArea.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrProcessAddOverwriteEvent);


	    	    
        }
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }

    }




    /**
     * @param _address
     */
    protected void processAddOverwriteEvent(WorkingMemoryChange _wmc) {


        try {

	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.m_address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof RobotPose) {
		lastPose = (RobotPose) data;
	    }
	    else if (data instanceof NavGraph) {
		lastNavGraph = (NavGraph) data;
	    }
	    else if (data instanceof ConvertPosition) {
		ConvertPosition cp = (ConvertPosition) data;
		String bufPosString = bufferPositionString(cp.m_x,
							   cp.m_y,
							   cp.m_bufferDistance);
		CELM_EventLocation location = new CELM_EventLocation(bufPosString);
		overwriteWorkingMemory(_wmc.m_address, 
				       new ConvertPosition(cp.m_x,
							   cp.m_y,
							   cp.m_bufferDistance,
							   location));
	    }
	    else if (data instanceof ConvertHere) {
		ConvertHere ch = (ConvertHere) data;
		
		String bufPosString;
		if (lastPose != null)
		    bufPosString = bufferPositionString(lastPose.m_x,
							lastPose.m_y,
							ch.m_bufferDistance);
		else {
		    log("warning: did not receive any RobotPose yet, " + 
			"so I assume we are still at (0, 0)"); 
		    bufPosString = bufferPositionString(0.0,
							0.0,
							ch.m_bufferDistance);
		}
		CELM_EventLocation location = new CELM_EventLocation(bufPosString);
		overwriteWorkingMemory(_wmc.m_address, 
				       new ConvertHere(ch.m_bufferDistance,
						       location));
	    }
	    else if (data instanceof ConvertArea) {

		// log("Sorry, ConvertArea is not implemented yet!!!");
		// System.err.println("\n\n\nSorry, ConvertArea is not implemented yet!!!\n\n\n");
		ConvertArea ca = (ConvertArea) data;
		
		CELM_EventLocation location = 
		    new CELM_EventLocation(approximateArea(ca.m_areaID));

		overwriteWorkingMemory(_wmc.m_address, 
				       new ConvertArea(ca.m_areaID,
						       location));
		
	    }



     	}    
 
        catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    if (GlobalSettings.exitOnException)
		System.exit(GlobalSettings.exitValueOnException);
        }
    }
    

    protected String bufferPositionString(double x, double y, double buffer) {
	double coordinates[] = new double[2];
	coordinates[0] = x;
	coordinates[1] = y;
	return elFactory.fromPoint(coordinates, buffer).getWKTString();
    }

    protected String approximateArea(long areaID) {

	Vector<EventLocation> locations = new Vector<EventLocation>();

	// FOR NOW THIS CODE IGNORES GATEWAY NODES WHICH ARE NOT MARKED AS
	// BELONGING TO THAT AREA!!!

	for (FNode n : lastNavGraph.m_FNodes)
	    if (n.m_areaID == areaID) {
		if (n.m_gateway == 1) {

		    // WARNING: THIS CODE IS UNTESTED!!!

		    double coordinates[] = new double[2];

		    // add one side of the doorframe
		    coordinates[0] = n.m_x + n.m_width * 0.5 * Math.sin(n.m_theta);
		    coordinates[1] = n.m_y + n.m_width * 0.5 * Math.cos(n.m_theta);
		    locations.add(elFactory.fromPoint(coordinates, gatewayBuffer));

		    // add the other side of the doorframe
		    coordinates[0] = n.m_x - n.m_width * 0.5 * Math.sin(n.m_theta);
		    coordinates[1] = n.m_y - n.m_width * 0.5 * Math.cos(n.m_theta);
		    locations.add(elFactory.fromPoint(coordinates, gatewayBuffer));

		    // add the gateway node position itself 
		    // but only with the usual atomic buffer
		    coordinates[0] = n.m_x;
		    coordinates[1] = n.m_y;
		    locations.add(elFactory.fromPoint(coordinates, 
						      GlobalSettings.defaultAtomicBuffer));
		}
		else {
		    double coordinates[] = new double[2];
		    coordinates[0] = n.m_x;
		    coordinates[1] = n.m_y;
		    locations.add(elFactory.fromPoint(coordinates, fNodeBuffer));
		}
	    }
		
	EventLocation[] elArray = new EventLocation[locations.size()];
	locations.toArray(elArray);
	return elFactory.fromSubEventLocations(elArray).getWKTString();
    }


    @Override
    protected void taskAdopted(String _taskID) {}

    
    @Override
    protected void taskRejected(String _taskID) {}


}