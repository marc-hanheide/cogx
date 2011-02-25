package manipulation.core.bham.panTiltConnector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.panTiltConnector.PanTiltConnector;
import manipulation.core.share.types.PTZPosition;
import manipulation.math.MathOperation;

import org.apache.log4j.Logger;

import ptz.PTZInterfacePrx;
import ptz.PTZInterfacePrxHelper;
import ptz.PTZPose;
import Ice.Identity;

public class BhamPanTiltConnector implements PanTiltConnector {

	Manipulator manipulator;

	private Logger logger = Logger.getLogger(this.getClass());
	
	private PTZInterfacePrx ptzInterface;

	public BhamPanTiltConnector(Manipulator manipulator) {
		this.manipulator = manipulator;

		Ice.Communicator ic = null;
		ic = Ice.Util.initialize();

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
	public PTZPosition getPoseDeg() {
		PTZPosition radPose = getPoseRad();

		return new PTZPosition(MathOperation.getDegree(radPose.getPan()),
				MathOperation.getDegree(radPose.getTilt()));
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public PTZPosition getPoseRad() {

		PTZPose pose = ptzInterface.getPose().pose;

		return new PTZPosition(pose.pan, pose.tilt);

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void setPoseDeg(PTZPosition position) {

		PTZPosition radPosition = new PTZPosition(MathOperation
				.getRadiant(position.getPan()), MathOperation
				.getRadiant(position.getTilt()));

		setPoseRad(radPosition);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void setPoseRad(PTZPosition position) {
		PTZPose newPose = new PTZPose(position.getPan(), position.getTilt(), 0);
		ptzInterface.setPose(newPose);
	}

}
