package patrol;

import ptz.PTZInterface;
import ptz.PTZInterfacePrx;
import ptz.PTZPose;
import cast.CASTException;
import cast.architecture.ManagedComponent;

public class HeadTurner extends ManagedComponent {

	private PTZInterfacePrx m_ptzServer;

	public HeadTurner() {

	}

	@Override
	protected void start() {
		try {
			m_ptzServer = getIceServer("ptz.server", PTZInterface.class,
					PTZInterfacePrx.class);
		} catch (CASTException e) {
			logException(e);
		}
	}

	@Override
	protected void runComponent() {

		double minPan = -Math.PI / 3;
		double panRange = Math.PI * 2 / 3;
		PTZPose pose = new PTZPose(0, 0, 0);

		while (isRunning() && m_ptzServer != null) {

			pose.pan = minPan + Math.random() * panRange;
			m_ptzServer.setPose(pose);
			sleepComponent((long) (10000 + Math.random() * 20000));
		}
	}
}
