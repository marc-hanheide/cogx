package manipulation.core.cogx;

import org.apache.log4j.Logger;

import manipulation.core.share.Manipulator;
import manipulation.core.share.ManipulatorPartFactory;
import manipulation.core.share.ManipulatorStore;
import manipulation.core.share.Manipulator.ManipulatorName;

/**
 * Store to order the manipulator for Birmingham / CogX
 * 
 * @author ttoenige
 * 
 */
public class CogXManipulatorStore extends ManipulatorStore {
	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected Manipulator createManipulator(ManipulatorName name) {
		Manipulator manipulator = null;
		ManipulatorPartFactory manPartFactory;

		switch (name) {
		case INTELLIGENT_GRASPING:
			logger.debug("Int grasp CogX");
			manPartFactory = new CogXManipulatorPartsFactory();
			manipulator = new CogXIntGraspingManipulator(manPartFactory);
			manipulator.setName(name);
			break;
		case MASTER_THESIS:
			logger.debug("Int grasp MasterThesis");
			 manPartFactory = new MasterThesisManipulatorPartsFactory();
			manipulator = new MasterThesisIntGraspingManipulator(manPartFactory);
			manipulator.setName(name);
			break;
		default:
			logger.debug("No name ....");
			break;
		}

		return manipulator;
	}

}
