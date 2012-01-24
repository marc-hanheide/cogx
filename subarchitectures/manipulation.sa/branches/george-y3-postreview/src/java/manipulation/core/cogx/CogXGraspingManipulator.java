package manipulation.core.cogx;

import org.apache.log4j.Logger;

import manipulation.core.share.Manipulator;
import manipulation.core.share.ManipulatorPartFactory;

/**
 * grasping manipulator for Birmingham / CogX
 * 
 * @author Torben Toeniges
 * 
 */
public class CogXGraspingManipulator extends Manipulator {
	ManipulatorPartFactory partFactory;

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor of the manipulator
	 * 
	 * @param partFactory
	 *            factory of the corresponding parts
	 */
	public CogXGraspingManipulator(ManipulatorPartFactory partFactory) {
		this.partFactory = partFactory;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void prepare() {
		logger.debug("Prepare " + this.getName().toString());

		setBaseConnector(partFactory.createBaseConnector(this));
		setCalibrationConnector(partFactory.createCalibrationConnector(this));
		setVirtualSceneConnector(partFactory.createVirtualSceneConnector(this));
		setArmConnector(partFactory.createArmConnector(this));
		setCamConnector(partFactory.createCamConnector(this));
	}

}