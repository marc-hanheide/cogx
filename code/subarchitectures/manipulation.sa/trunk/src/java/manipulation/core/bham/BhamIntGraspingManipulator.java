package manipulation.core.bham;

import org.apache.log4j.Logger;

import manipulation.core.share.Manipulator;
import manipulation.core.share.ManipulatorPartFactory;

/**
 * grasping manipulator for Birmingham / CogX
 * 
 * @author ttoenige
 * 
 */
public class BhamIntGraspingManipulator extends Manipulator {
	ManipulatorPartFactory partFactory;

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor of the manipulator
	 * 
	 * @param partFactory
	 *            factory of the corresponding parts
	 */
	public BhamIntGraspingManipulator(ManipulatorPartFactory partFactory) {
		this.partFactory = partFactory;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected void prepare() {
		logger.debug("Prepare " + this.getName().toString());

	
		setBaseConnector(partFactory.createBaseConnector(this));
		setSimulationConnector(partFactory.createSimulationConnector(this));
		setArmConnector(partFactory.createArmConnector(this));
		setCamConnector(partFactory.createCamConnector(this));
		setMapConnector(partFactory.createMapConnector(this));
		setCalibrationConnector(partFactory.createCalibrationConnector(this));

		setPanTiltConnector(partFactory.createPanTiltConnector(this));

	}

}