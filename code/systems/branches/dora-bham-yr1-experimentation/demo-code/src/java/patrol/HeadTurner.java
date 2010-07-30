package patrol;

import ptz.PTZInterfacePrx;
import ptz.PTZInterfacePrxHelper;
import ptz.PTZPose;
import Ice.Communicator;
import Ice.Identity;
import Ice.ObjectPrx;
import cast.architecture.ManagedComponent;
import cast.cdl.CPPSERVERPORT;

public class HeadTurner extends ManagedComponent {

	private PTZInterfacePrx m_ptzServer;

	public HeadTurner() {

	}

	@Override
	protected void start() {
		try {

			Communicator ic = Ice.Util.initialize();

			Identity id = new Identity("PTZServer", "PTZServer");

			StringBuilder sb = new StringBuilder(ic.identityToString(id));
			sb.append(":default -h localhost -p ");
			sb.append(CPPSERVERPORT.value);

			ObjectPrx base = ic.stringToProxy(sb.toString());
			m_ptzServer = PTZInterfacePrxHelper.uncheckedCast(base);
		} catch (Exception e) {
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
