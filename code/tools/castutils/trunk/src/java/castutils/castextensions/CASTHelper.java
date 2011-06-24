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
	private Logger logger = null;
	
	protected CASTHelper(ManagedComponent c) {
		component = c;
		name = this.getClass().getSimpleName();
	}

	protected Logger getLogger() {
		if (logger == null) {
			if (component.getLoggerName().equals("null.null"))
				return Logger.getLogger(name);
			else
				logger = Logger.getLogger(component.getLoggerName() + "."
						+ name);
		} 
		return logger;
	}

	protected void println(Object o) {
		getLogger().info(name + ": " + o);
	}

	protected void log(Object o) {
		getLogger().debug(name + ": " + o);
	}

	protected void debug(Object o) {
		getLogger().trace(name + ": " + o);
	}
}
