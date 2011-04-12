package manipulation.core.cogx;

import manipulation.core.cogx.armConnector.CogXKatanaArmConnector;
import manipulation.core.cogx.baseConnector.MasterThesisDoraBaseConnector;
import manipulation.core.cogx.baseConnector.MasterThesisLocalMapConnector;
import manipulation.core.cogx.calibrationConnector.HomographyCalibration;
import manipulation.core.cogx.camConnector.MasterThesisBlortConnector;
import manipulation.core.cogx.panTiltConnector.CogXPanTiltConnector;
import manipulation.core.cogx.virtualSceneConnector.CogXVirtualSceneConnector;
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
public class MasterThesisManipulatorPartsFactory implements
		ManipulatorPartFactory {

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
		logger.debug("Creating BaseConnector: MasterThesisBaseCon");
		return new MasterThesisDoraBaseConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public CamConnector createCamConnector(Manipulator manipulator) {
		logger.debug("Creating CamConnector: MasterThesisBlortConnector");
		return new MasterThesisBlortConnector(manipulator);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public MapConnector createMapConnector(Manipulator manipulator) {
		logger.debug("Creating MapConnector: MasterThesisMapCon");
		return new MasterThesisLocalMapConnector(manipulator);
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
