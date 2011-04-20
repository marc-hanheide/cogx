package manipulation.muster.core.cogx;

import manipulation.muster.core.cogx.armConnector.CogXKatanaArmConnector;
import manipulation.muster.core.cogx.baseConnector.CogXDoraBaseConnector;
import manipulation.muster.core.cogx.baseConnector.CogXLocalMapConnector;
import manipulation.muster.core.cogx.calibrationConnector.CogXCalibConnector;
import manipulation.muster.core.cogx.camConnector.CogXBlortConnector;
import manipulation.muster.core.cogx.panTiltConnector.CogXPanTiltConnector;
import manipulation.muster.core.cogx.virtualSceneConnector.CogXVirtualSceneConnector;
import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.ManipulatorPartFactory;
import manipulation.muster.core.share.armConnector.ArmConnector;
import manipulation.muster.core.share.baseConnector.BaseConnector;
import manipulation.muster.core.share.baseConnector.MapConnector;
import manipulation.muster.core.share.calibrationConnector.CalibrationConnector;
import manipulation.muster.core.share.camConnector.CamConnector;
import manipulation.muster.core.share.panTiltConnector.PanTiltConnector;
import manipulation.muster.core.share.virtualSceneConnector.VirtualSceneConnector;

import org.apache.log4j.Logger;

/**
 * factory to create the parts of a manipulator for Birmingham / CogX
 * 
 * @author ttoenige
 * 
 */
public class CogXManipulatorPartsFactory implements ManipulatorPartFactory {

	private static Logger logger = Logger
			.getLogger("manipulation.core.cogx.CogXManipulatiorPartFactory");

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ArmConnector createArmConnector(Manipulator manipulator) {
		logger.debug("Create ArmConnector: CogXKatanaArmConnector");
		return new CogXKatanaArmConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public BaseConnector createBaseConnector(Manipulator manipulator) {
		logger.debug("Creating BaseConnector: CogXDoraBaseConnector");
		return new CogXDoraBaseConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public CamConnector createCamConnector(Manipulator manipulator) {
		logger.debug("Creating CamConnector: CogXBlortConnector");
		return new CogXBlortConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public MapConnector createMapConnector(Manipulator manipulator) {
		logger.debug("Creating MapConnector: CogXLocalMapConnector");
		return new CogXLocalMapConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public VirtualSceneConnector createVirtualSceneConnector(
			Manipulator manipulator) {
		logger.debug("Creating SimulationConnector: CogXSimulationConnector");
		return new CogXVirtualSceneConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public CalibrationConnector createCalibrationConnector(
			Manipulator manipulator) {
		logger.debug("Creating CalibrationConnector: CogXCalibConnector");
		return new CogXCalibConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public PanTiltConnector createPanTiltConnector(Manipulator manipulator) {
		logger.debug("Creating PanTiltConnecotr: BHamPanTiltConnector");
		return new CogXPanTiltConnector(manipulator);
	}

}
