package manipulation.muster.core.cogx;

import org.apache.log4j.Logger;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.ManipulatorPartFactory;
import manipulation.muster.core.share.ManipulatorStore;
import manipulation.muster.core.share.Manipulator.ManipulatorName;

/**
 * Store to order the manipulator for Birmingham / CogX
 * 
 * @author ttoenige
 * 
 */
public class ExemplarySolutionManipulatorStore extends ManipulatorStore {
	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * {@inheritDoc}
	 */
	@Override
	protected Manipulator createManipulator(ManipulatorName name) {
		Manipulator manipulator = null;
		ManipulatorPartFactory manPartFactory;

		switch (name) {
		case EXEMPLARY_SOLUTION:
			logger.debug("Int grasp exemplay solution");
			manPartFactory = new ExemplarySolutionManipulatorPartsFactory();
			manipulator = new ExemplarySolutionIntGraspingManipulator(
					manPartFactory);
			manipulator.setName(name);
			break;
		default:
			logger.debug("No name ....");
			break;
		}

		return manipulator;
	}

}
