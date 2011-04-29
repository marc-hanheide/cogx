package manipulation.core.share;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.core.share.types.Configuration;
import manipulation.itemMemory.ItemMemory;
import manipulation.runner.share.Runner;

import org.apache.log4j.Logger;

/**
 * store to order the manipulator
 * 
 * @author Torben Toeniges
 * 
 */
public abstract class ManipulatorStore {
	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * creates a manipulator with the given name
	 * 
	 * @param name
	 *            given name
	 * @return created manipulator
	 */
	protected abstract Manipulator createManipulator(
			Manipulator.ManipulatorName name);

	/**
	 * orders a manipulator
	 * 
	 * @param name
	 *            name of the manipulator to order
	 * @param runner
	 *            current runner
	 * @param itemMemory
	 *            current item memory
	 * @param watcher
	 *            current command watcher
	 * @param configuration
	 *            current configuration
	 * @return ordered manipulator
	 */
	public Manipulator orderManipulator(Manipulator.ManipulatorName name,
			Runner runner, ItemMemory itemMemory, CommandWatcher watcher,
			Configuration configuration) {
		Manipulator manipulator = createManipulator(name);
		logger.debug("Making a " + manipulator.getName().toString());

		manipulator.setConfiguration(configuration);
		manipulator.setItemMemory(itemMemory);
		manipulator.setWatcher(watcher);
		manipulator.setRunner(runner);

		manipulator.prepare();

		return manipulator;

	}
}
