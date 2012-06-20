package eu.cogx.ptz;

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
import castutils.castextensions.WMEventQueue;

public class PanTiltZoomServer extends ManagedComponent {

	private static final String KEY_PAN = "--pan";

	private static final String KEY_TILT = "--tilt";

	private static final int TIMEOUT_MS = 8000;

	private static final int TIME_WAIT_MS = 100;

	private static final double TOLERANCE = 0.001;

	private static final int MAX_LOOPS = TIMEOUT_MS / TIME_WAIT_MS;

	private static final PTZPose[] FIXED_INIT_POSES = new PTZPose[] {
			new PTZPose(45 * Math.PI / 180, 25 * Math.PI / 180, 0),
			new PTZPose(-45 * Math.PI / 180, -25 * Math.PI / 180, 0) };

	private Logger logger = Logger.getLogger(this.getClass());

	private PTZInterfacePrx ptzInterface = null;

	private String ptzServerComponent;

	private boolean doTheShake;

	private double startPan;

	private double startTilt;

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
		double panError = 1 - (Math.sin(pos1.pan) * Math.sin(pos2.pan) + Math
				.cos(pos1.pan)
				* Math.cos(pos2.pan));
		double tiltError = 1 - (Math.sin(pos1.tilt) * Math.sin(pos2.tilt) + Math
				.cos(pos1.tilt)
				* Math.cos(pos2.tilt));
		return panError + tiltError;
	}

	private void movePTZ(WorkingMemoryChange _wmc) {
		try {
			SetPTZPoseCommand cmd = getMemoryEntry(_wmc.address,
					SetPTZPoseCommand.class);

			logger.debug("moving to pose: " + cmd.pose.pan + ", "
					+ cmd.pose.tilt);

			double different = Double.MAX_VALUE;

			PTZPose currentPose = null;

			int loops = 0;
			while (different >= TOLERANCE) {
				ptzInterface.setPose(cmd.pose);
				try {
					Thread.sleep(TIME_WAIT_MS);
					if (loops++ > MAX_LOOPS)
						break;
				} catch (InterruptedException e) {
					logger.error(e);
				}
				currentPose = ptzInterface.getPose().pose;
				different = ptzPosError(cmd.pose, currentPose);
				logger.debug("Pan-Tilt motion diff: " + different);
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

		if (config.containsKey("--shake")) {
			doTheShake = true;
		} else {
			doTheShake = false;
		}

		if (config.containsKey(KEY_PAN)) {
			startPan = Double.parseDouble(config.get(KEY_PAN));
		} else {
			startPan = 0.0;
		}
		if (config.containsKey(KEY_TILT)) {
			startTilt = Double.parseDouble(config.get(KEY_TILT));
		} else {
			startTilt = 0.0;
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
		//lockComponent();
		if (doTheShake) {
			println("go through a sequence of commands to help with a pan-tilt player bug, that seems to choke when the pan-tilt is not at 0.0 at start up");
			for (PTZPose pose : FIXED_INIT_POSES) {
				SetPTZPoseCommand cmd = new SetPTZPoseCommand(pose,
						PTZCompletion.COMPINIT);
				String id = newDataID();
				WMEventQueue queue = new WMEventQueue();
				addChangeFilter(ChangeFilterFactory.createIDFilter(id,
						WorkingMemoryOperation.OVERWRITE), queue);
				try {
					addToWorkingMemory(id, cmd);
					queue.take();
					removeChangeFilter(queue);
					deleteFromWorkingMemory(id);
				} catch (CASTException e) {
					logException(e);
				} catch (InterruptedException e) {
					logException(e);
				}
			}
		}
		// go to initial pose
		SetPTZPoseCommand cmd = new SetPTZPoseCommand(new PTZPose(startPan,
				startTilt, 0.0), PTZCompletion.COMPINIT);
		String id = newDataID();
		WMEventQueue queue = new WMEventQueue();
		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.OVERWRITE), queue);
		try {
			addToWorkingMemory(id, cmd);
			queue.take();
			removeChangeFilter(queue);
			deleteFromWorkingMemory(id);
		} catch (CASTException e) {
			logException(e);
		} catch (InterruptedException e) {
			logException(e);
		}

		//unlockComponent();
	}
}
