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
import celm.autogen.CELEventLocation;
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
    protected RobotPose2d            lastPose       = null;
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
			    createGlobalTypeFilter(RobotPose2d.class, 
						   WorkingMemoryOperation.ADD), 
			    wmcrProcessAddOverwriteEvent);
	    addChangeFilter(ChangeFilterFactory.
			    createGlobalTypeFilter(RobotPose2d.class, 
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

	    CASTData<?> wme = getWorkingMemoryEntry(_wmc.address);
	    
	    Object data = wme.getData();
	    
	    if (data instanceof RobotPose2d) {
		lastPose = (RobotPose2d) data;
	    }
	    else if (data instanceof NavGraph) {
		lastNavGraph = (NavGraph) data;
	    }
	    else if (data instanceof ConvertPosition) {
		ConvertPosition cp = (ConvertPosition) data;
		String bufPosString = bufferPositionString(cp.x,
							   cp.y,
							   cp.bufferDistance);
		CELEventLocation location = new CELEventLocation(bufPosString);
		overwriteWorkingMemory(_wmc.address, 
				       new ConvertPosition(cp.x,
							   cp.y,
							   cp.bufferDistance,
							   location));
	    }
	    else if (data instanceof ConvertHere) {
		ConvertHere ch = (ConvertHere) data;
		
		String bufPosString;
		if (lastPose != null)
		    bufPosString = bufferPositionString(lastPose.x,
							lastPose.y,
							ch.bufferDistance);
		else {
		    log("warning: did not receive any RobotPose2d yet, " + 
			"so I assume we are still at (0, 0)"); 
		    bufPosString = bufferPositionString(0.0,
							0.0,
							ch.bufferDistance);
		}
		CELEventLocation location = new CELEventLocation(bufPosString);
		overwriteWorkingMemory(_wmc.address, 
				       new ConvertHere(ch.bufferDistance,
						       location));
	    }
	    else if (data instanceof ConvertArea) {

		// log("Sorry, ConvertArea is not implemented yet!!!");
		// System.err.println("\n\n\nSorry, ConvertArea is not implemented yet!!!\n\n\n");
		ConvertArea ca = (ConvertArea) data;
		
		CELEventLocation location = 
		    new CELEventLocation(approximateArea(ca.areaID));

		overwriteWorkingMemory(_wmc.address, 
				       new ConvertArea(ca.areaID,
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

	for (FNode n : lastNavGraph.FNodes)
	    if (n.areaID == areaID) {
		if (n.gateway == 1) {

		    // WARNING: THIS CODE IS UNTESTED!!!

		    double coordinates[] = new double[2];

		    // add one side of the doorframe
		    coordinates[0] = n.x + n.width * 0.5 * Math.sin(n.theta);
		    coordinates[1] = n.y + n.width * 0.5 * Math.cos(n.theta);
		    locations.add(elFactory.fromPoint(coordinates, gatewayBuffer));

		    // add the other side of the doorframe
		    coordinates[0] = n.x - n.width * 0.5 * Math.sin(n.theta);
		    coordinates[1] = n.y - n.width * 0.5 * Math.cos(n.theta);
		    locations.add(elFactory.fromPoint(coordinates, gatewayBuffer));

		    // add the gateway node position itself 
		    // but only with the usual atomic buffer
		    coordinates[0] = n.x;
		    coordinates[1] = n.y;
		    locations.add(elFactory.fromPoint(coordinates, 
						      GlobalSettings.defaultAtomicBuffer));
		}
		else {
		    double coordinates[] = new double[2];
		    coordinates[0] = n.x;
		    coordinates[1] = n.y;
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
