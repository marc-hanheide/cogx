package ptz.ptzServer;

import java.util.Map;

import org.apache.log4j.Logger;

import ptz.GetPTZPoseCommand;
import ptz.PTZCompletion;
import ptz.PTZInterfacePrx;
import ptz.PTZInterfacePrxHelper;
import ptz.PTZPose;
import ptz.SetPTZPoseCommand;
import Ice.Identity;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class PanTiltZoomServer extends ManagedComponent {

	private Logger logger = Logger.getLogger(this.getClass());

	private PTZInterfacePrx ptzInterface = null;

	private void addPanTiltCommandListener() {
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				SetPTZPoseCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						movePTZ(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				GetPTZPoseCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						sendPTZPoseToMemory(_wmc);
					}
				});
	}

	private void sendPTZPoseToMemory(WorkingMemoryChange _wmc) {
		try {
			GetPTZPoseCommand cmd = getMemoryEntry(_wmc.address,
					GetPTZPoseCommand.class);
			cmd.pose = ptzInterface.getPose().pose;
			cmd.comp = PTZCompletion.SUCCEEDED;
			overwriteWorkingMemory(_wmc.address, cmd);
		} catch (CASTException e) {
			logger.error(e);
		}

	}

	private double ptzPosError(PTZPose pos1, PTZPose pos2) {
		return Math.abs(pos1.pan - pos2.pan)
				+ Math.abs(pos1.tilt - pos2.tilt) 
				+ Math.abs(pos1.zoom - pos2.zoom);
	}

	private void movePTZ(WorkingMemoryChange _wmc) {
		try {
			SetPTZPoseCommand cmd = getMemoryEntry(_wmc.address,
					SetPTZPoseCommand.class);

			PTZPose originalPose = ptzInterface.getPose().pose;

			ptzInterface.setPose(cmd.pose);

			double different = Double.MAX_VALUE;

			PTZPose currentPose = null;
			PTZPose oldPose = new PTZPose(Double.MAX_VALUE, Double.MAX_VALUE, 0);
			while (different > 0.01) {
				currentPose = ptzInterface.getPose().pose;
				different = ptzPosError(currentPose, oldPose);
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					logger.error(e);
				}

				oldPose = currentPose;
			}

			if (ptzPosError(originalPose, currentPose) != 0) {
				cmd.pose = ptzInterface.getPose().pose;
				cmd.comp = PTZCompletion.SUCCEEDED;
			} else {
				cmd.comp = PTZCompletion.FAILED;
			}

			overwriteWorkingMemory(_wmc.address, cmd);

		} catch (CASTException e) {
			logger.error(e);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void configure(Map<String, String> config) {

		if (config.containsKey("--testGUI")) {
			new PanTiltZoomGUI(this);		
		}

		Ice.Communicator ic = Ice.Util.initialize();

		Identity id = new Identity();
		id.name = "PTZServer";
		id.category = "PTZServer";

		String path = ic.identityToString(id) + ":default -h localhost -p "
				+ cast.cdl.CPPSERVERPORT.value;

		Ice.ObjectPrx init = ic.stringToProxy(path);
		ptzInterface = PTZInterfacePrxHelper.uncheckedCast(init);



	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void start() {
		addPanTiltCommandListener();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void runComponent() {
	}
}
