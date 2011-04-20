package manipulation.muster.core.cogx;

import manipulation.muster.core.cogx.armConnector.ExemplarySolutionKatanaArmConnector;
import manipulation.muster.core.cogx.baseConnector.ExemplarySolutionDoraBaseConnector;
import manipulation.muster.core.cogx.baseConnector.ExemplarySolutionLocalMapConnector;
import manipulation.muster.core.cogx.calibrationConnector.ExemplarySolutionCalibConnector;
import manipulation.muster.core.cogx.camConnector.ExemplarySolutionBlortConnector;
import manipulation.muster.core.cogx.panTiltConnector.ExemplarySolutionPanTiltConnector;
import manipulation.muster.core.cogx.virtualSceneConnector.ExemplarySolutionVirtualSceneConnector;
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
		return new ExemplarySolutionCalibConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public PanTiltConnector createPanTiltConnector(Manipulator manipulator) {
		logger.debug("Creating PanTiltConnecotr: BHamPanTiltConnector");
		return new ExemplarySolutionPanTiltConnector(manipulator);
	}

}
