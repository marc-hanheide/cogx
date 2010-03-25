/**
 * 
 */
package castutils.castextensions;

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
	}
	
	protected void println(Object o) {
		logger = Logger.getLogger(component.getLoggerName()+"."+name);
		logger.info(name + ": "+ o);
	}
	protected void log(Object o) {
		logger = Logger.getLogger(component.getLoggerName()+"."+name);
		logger.debug(name + ": "+ o);
	}
	protected void debug(Object o) {
		logger = Logger.getLogger(component.getLoggerName()+"."+name);
		logger.trace(name + ": "+ o);
	}
}
