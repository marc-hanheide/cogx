package manipulation.core.cogx;

import manipulation.core.cogx.armConnector.CogXKatanaArmConnector;
import manipulation.core.cogx.baseConnector.CogXDoraBaseConnector;
import manipulation.core.cogx.baseConnector.CogXLocalMapConnector;
import manipulation.core.cogx.calibrationConnector.HomographyCalibration;
import manipulation.core.cogx.camConnector.CogXBlortConnector;
import manipulation.core.cogx.panTiltConnector.CogXPanTiltConnector;
import manipulation.core.cogx.simulationConnector.CogXSimulationConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.ManipulatorPartFactory;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.baseConnector.BaseConnector;
import manipulation.core.share.baseConnector.MapConnector;
import manipulation.core.share.calibrationConnector.CalibrationConnector;
import manipulation.core.share.camConnector.CamConnector;
import manipulation.core.share.panTiltConnector.PanTiltConnector;
import manipulation.core.share.simulationConnector.SimulationConnector;

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
	public SimulationConnector createSimulationConnector(Manipulator manipulator) {
		logger.debug("Creating SimulationConnector: CogXSimulationConnector");
		return new CogXSimulationConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public CalibrationConnector createCalibrationConnector(
			Manipulator manipulator) {
		logger.debug("Creating CalibrationConnector: HomographyCalibration");
		return new HomographyCalibration(manipulator);
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
