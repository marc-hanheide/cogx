package manipulation.core.cogx;

import manipulation.core.cogx.armConnector.CogXKatanaArmConnector;
import manipulation.core.cogx.baseConnector.CogXDoraBaseConnector;
import manipulation.core.cogx.calibrationConnector.CogXCalibConnector;
import manipulation.core.cogx.camConnector.CogXBlortConnector;
import manipulation.core.cogx.virtualSceneConnector.CogXVirtualSceneConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.ManipulatorPartFactory;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.baseConnector.BaseConnector;
import manipulation.core.share.calibrationConnector.CalibrationConnector;
import manipulation.core.share.camConnector.CamConnector;
import manipulation.core.share.virtualSceneConnector.VirtualSceneConnector;

import org.apache.log4j.Logger;

/**
 * factory to create the different parts of a manipulator for Birmingham / CogX
 * 
 * @author Torben Toeniges
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

}
