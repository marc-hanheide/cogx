package manipulation.core.bham;

import manipulation.core.bham.armConnector.BhamKatanaArmConnector;
import manipulation.core.bham.baseConnector.BhamDoraBaseConnector;
import manipulation.core.bham.baseConnector.BhamLocalMapConnector;
import manipulation.core.bham.calibrationConnector.HomographyCalibration;
import manipulation.core.bham.camConnector.BhamBlortConnector;
import manipulation.core.bham.panTiltConnector.BhamPanTiltConnector;
import manipulation.core.bham.simulationConnector.BhamSimulationConnector;
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
public class BhamManipulatorPartsFactory implements ManipulatorPartFactory {

	private static Logger logger = Logger
			.getLogger("manipulation.core.bham.BhamManipulatiorPartFactory");

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ArmConnector createArmConnector(Manipulator manipulator) {
		logger.debug("Create ArmConnector: BhamKatanaArmConnector");
		return new BhamKatanaArmConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public BaseConnector createBaseConnector(Manipulator manipulator) {
		logger.debug("Creating BaseConnector: BhamDoraBaseConnector");
		return new BhamDoraBaseConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public CamConnector createCamConnector(Manipulator manipulator) {
		logger.debug("Creating CamConnector: BhamBlortConnector");
		return new BhamBlortConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public MapConnector createMapConnector(Manipulator manipulator) {
		logger.debug("Creating MapConnector: BhamLocalMapConnector");
		return new BhamLocalMapConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public SimulationConnector createSimulationConnector(Manipulator manipulator) {
		logger.debug("Creating SimulationConnector: BhamSimulationConnector");
		return new BhamSimulationConnector(manipulator);
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
		return new BhamPanTiltConnector(manipulator);
	}

}
