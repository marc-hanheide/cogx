package manipulation.core.cogx;

import manipulation.core.cogx.armConnector.ExemplarySolutionKatanaArmConnector;
import manipulation.core.cogx.baseConnector.ExemplarySolutionDoraBaseConnector;
import manipulation.core.cogx.baseConnector.ExemplarySolutionLocalMapConnector;
import manipulation.core.cogx.calibrationConnector.CogXCalibConnector;
import manipulation.core.cogx.calibrationConnector.HomographyCalibration;
import manipulation.core.cogx.camConnector.ExemplarySolutionBlortConnector;
import manipulation.core.cogx.panTiltConnector.CogXPanTiltConnector;
import manipulation.core.cogx.virtualSceneConnector.ExemplarySolutionVirtualSceneConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.ManipulatorPartFactory;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.baseConnector.BaseConnector;
import manipulation.core.share.baseConnector.MapConnector;
import manipulation.core.share.calibrationConnector.CalibrationConnector;
import manipulation.core.share.camConnector.CamConnector;
import manipulation.core.share.panTiltConnector.PanTiltConnector;
import manipulation.core.share.virtualSceneConnector.VirtualSceneConnector;

import org.apache.log4j.Logger;

/**
 * factory to create the parts of a manipulator for Birmingham / CogX
 * 
 * @author ttoenige
 * 
 */
public class ExemplarySolutionManipulatorPartsFactory implements
		ManipulatorPartFactory {

	private static Logger logger = Logger
			.getLogger("manipulation.core.cogx.ExemplarySolutionFactory");

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ArmConnector createArmConnector(Manipulator manipulator) {
		logger.debug("Create ArmConnector: ExemplarySolutionKatanaArmConnector");
		return new ExemplarySolutionKatanaArmConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public BaseConnector createBaseConnector(Manipulator manipulator) {
		logger.debug("Creating BaseConnector: ExemplarySolutionDoraBaseConnector");
		return new ExemplarySolutionDoraBaseConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public CamConnector createCamConnector(Manipulator manipulator) {
		logger.debug("Creating CamConnector: ExemplarySolutionBlortConnector");
		return new ExemplarySolutionBlortConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public MapConnector createMapConnector(Manipulator manipulator) {
		logger.debug("Creating MapConnector: ExemplarySolutionLocalMapConnector");
		return new ExemplarySolutionLocalMapConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public VirtualSceneConnector createVirtualSceneConnector(
			Manipulator manipulator) {
		logger.debug("Creating SimulationConnector: ExemplarySolutionVirtualSceneConnector");
		return new ExemplarySolutionVirtualSceneConnector(manipulator);
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
