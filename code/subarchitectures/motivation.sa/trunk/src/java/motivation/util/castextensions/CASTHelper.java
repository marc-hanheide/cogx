/**
 * 
 */
package motivation.util.castextensions;

import org.apache.log4j.Logger;

import cast.architecture.ManagedComponent;

/**
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
		logger = Logger.getLogger(c.getLoggerName()+"."+name);
	}
	
	protected void println(Object o) {
		logger.info(name + ": "+ o);
	}
	protected void log(Object o) {
		logger.debug(name + ": "+ o);
	}
	protected void debug(Object o) {
		logger.trace(name + ": "+ o);
	}
}
