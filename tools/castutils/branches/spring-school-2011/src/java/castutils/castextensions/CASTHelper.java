/**
 * 
 */
package castutils.castextensions;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;

/**
 * a helper class for all classes that need access to the memory but are not
 * derived from a {@link ManagedComponent}.
 * 
 * @author marc
 * 
 */
public class CASTHelper {
	protected ManagedComponent component;
	protected String name;
	protected Logger logger;

	protected CASTHelper(ManagedComponent c) {
		component = c;
		name = this.getClass().getSimpleName();
		logger = Logger.getLogger(component.getLoggerName() + "." + name);
	}

	protected void println(Object o) {
		logger.info(name + ": " + o);
	}

	protected void log(Object o) {
		logger.debug(name + ": " + o);
	}

	protected void debug(Object o) {
		logger.trace(name + ": " + o);
	}
}
