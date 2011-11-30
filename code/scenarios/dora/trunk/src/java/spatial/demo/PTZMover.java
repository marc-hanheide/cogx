package spatial.demo;

import org.apache.log4j.pattern.LogEvent;

import ptz.PTZCompletion;
import ptz.PTZPose;
import ptz.SetPTZPoseCommand;
import cast.CASTException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEventQueue;

public class PTZMover {
	private static final double PAN_RANGE = 60;
	private static final double TILT_RANGE = 20;
	final ManagedComponent component;

	public PTZMover(ManagedComponent component) {
		super();
		this.component = component;
	}

	/**
	 * moves to a random pose and blocks until it's reached.
	 */
	public void moveToRandomPose() {
		double pan = (Math.random() - 0.5) * (PAN_RANGE * Math.PI / 180);
		double tilt = (Math.random() - 0.5) * (TILT_RANGE * Math.PI / 180);

		runPanTiltCmd(pan, tilt);

	}

	private void runPanTiltCmd(double pan, double tilt) {
		try {
			SetPTZPoseCommand cmd = new SetPTZPoseCommand(new PTZPose(pan,
					tilt, 1), PTZCompletion.COMPINIT);
			WorkingMemoryAddress wma = new WorkingMemoryAddress(
					component.newDataID(), component.getSubarchitectureID());
			WMEventQueue queue = new WMEventQueue();
			component.addChangeFilter(
					ChangeFilterFactory.createAddressFilter(wma), queue);
			component.addToWorkingMemory(wma, cmd);	
			while (cmd.comp == PTZCompletion.COMPINIT) {
				WorkingMemoryChange wmc = queue.take();
				cmd = component.getMemoryEntry(wmc.address,
						SetPTZPoseCommand.class);
			}
			component.removeChangeFilter(queue);
			component.deleteFromWorkingMemory(wma);
		} catch (InterruptedException e) {
			component.logException(e);
		} catch (CASTException e) {
			component.logException(e);
		}

	}
}
