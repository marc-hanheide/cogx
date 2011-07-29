package ptz.ptzServer;

import java.util.Map;

import org.apache.log4j.Logger;

import ptz.GetPTZPoseCommand;
import ptz.PTZCompletion;
import ptz.PTZInterface;
import ptz.PTZInterfacePrx;
import ptz.PTZPose;
import ptz.SetPTZPoseCommand;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class PanTiltZoomServer extends ManagedComponent {

	private static final int TIMEOUT_MS = 10000;

	private static final int TIME_WAIT_MS = 200;

	private static final double TOLERANCE = 0.05;

	private static final int MAX_LOOPS = TIMEOUT_MS / TIME_WAIT_MS;

	private Logger logger = Logger.getLogger(this.getClass());

	private PTZInterfacePrx ptzInterface = null;

	private String ptzServerComponent;

	private void addPanTiltCommandListener() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				SetPTZPoseCommand.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						movePTZ(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
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
		return Math.abs(pos1.pan - pos2.pan) + Math.abs(pos1.tilt - pos2.tilt)
				+ Math.abs(pos1.zoom - pos2.zoom);
	}

	private void movePTZ(WorkingMemoryChange _wmc) {
		try {
			SetPTZPoseCommand cmd = getMemoryEntry(_wmc.address,
					SetPTZPoseCommand.class);

			ptzInterface.setPose(cmd.pose);

			double different = Double.MAX_VALUE;

			PTZPose currentPose = null;
			
			int loops = 0;
			while (different >= TOLERANCE) {
				currentPose = ptzInterface.getPose().pose;
				different = ptzPosError(cmd.pose, currentPose);
				logger.debug("Pan-Tilt motion diff: " + different);
				try {
					Thread.sleep(TIME_WAIT_MS);
					if (loops > MAX_LOOPS)
						break;
				} catch (InterruptedException e) {
					logger.error(e);
				}
			}

			if (ptzPosError(cmd.pose, currentPose) < TOLERANCE) {
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

		ptzServerComponent = config.get("--ptzserver");
		if (ptzServerComponent == null) {
			ptzServerComponent = "ptz.server";
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void start() {
		try {
			ptzInterface = getIceServer(ptzServerComponent, PTZInterface.class,
					PTZInterfacePrx.class);
		} catch (CASTException e) {
			throw new RuntimeException("failed to get ptz interface", e);
		}
		addPanTiltCommandListener();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void runComponent() {
	}
}
