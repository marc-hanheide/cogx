/**
 * 
 */
package castutils.experimentation;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import org.apache.log4j.Logger;

import Ice.ObjectImpl;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.IceXMLSerializer;

/**
 * @author marc
 * 
 */
public class WMLogger extends ManagedComponent {

	private Set<Class<? extends Ice.ObjectImpl>> subscribedClasses = new HashSet<Class<? extends ObjectImpl>>();

	@Override
	public void start() /* throws UnknownSubarchitectureException */{
		// create log
		getLogger().debug("WMLogger Started");
		// register listeners
		for (Class<? extends Ice.ObjectImpl> cl : subscribedClasses) {
			register(cl);
		}
	}

	@Override
	protected void configure(Map<String, String> map) {
		super.configure(map);

		// Get subscriptions
		String subs = map.get("--subscribe");
		parseSubscriptions(subs);
	}

	/**
	 * Takes a list of classes as a string and looks up and loads them. They are
	 * added to the list SUBSCRIBED_CLASSES and in the start() method every
	 * class in this list has a listener added for it.
	 * 
	 * @param subs
	 *            comma seperated list of classes to listen for on working
	 *            memory
	 * @see #start()
	 * @see #subscribedClasses
	 */
	@SuppressWarnings("unchecked")
	private void parseSubscriptions(String subs) {
		String[] subsArr = subs.split("\\s*,\\s*"); // regex =
		// [whitespace][comma][whitespace]
		for (String className : subsArr) {
			try {
				getLogger().debug("Subscribtion: " + className);
				ClassLoader.getSystemClassLoader().loadClass(className);
				Class<? extends ObjectImpl> subscription = (Class<? extends ObjectImpl>) Class
						.forName(className); // TODO: Can this cast be tidier?
				subscribedClasses.add(subscription);
			} catch (ClassNotFoundException e) {
				String error = "trying to register for a class that doesn't exist. ["
						+ className + "]";
				getLogger().error(error); // log4j
				println(error);
				e.printStackTrace();
			}
		}
	}

	public <T extends Ice.ObjectImpl> void register(Class<T> type) {
		getLogger().trace("register listeners");
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(type,
				WorkingMemoryOperation.ADD), new LoggingChangeReceiver<T>(type), ChangeReceiverPriority.HIGH);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(type,
				WorkingMemoryOperation.DELETE), new LoggingChangeReceiver<T>(
				type), ChangeReceiverPriority.HIGH);
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(type,
				WorkingMemoryOperation.OVERWRITE),
				new LoggingChangeReceiver<T>(type), ChangeReceiverPriority.HIGH);
	}

	public class LoggingChangeReceiver<T extends Ice.ObjectImpl> implements
			WorkingMemoryChangeReceiver {

		/** The type of object this ChangeReciever is interested in */
		Class<T> specClass;

		final Logger logger;

		public LoggingChangeReceiver(Class<T> specClass) {
			super();
			this.specClass = specClass;
			this.logger = Logger.getLogger("WMLogger."+specClass.getCanonicalName());
		}

		@Override
		public synchronized void workingMemoryChanged(WorkingMemoryChange _wmc)
				throws CASTException {
			try {
			T m;
			String wmcXML = IceXMLSerializer.toXMLString(_wmc);
			String contentXML = "";
			switch (_wmc.operation) {
			case ADD:
			case OVERWRITE:
				m = getMemoryEntry(_wmc.address, specClass);
				contentXML = IceXMLSerializer.toXMLString(m);
				break;

			case DELETE:
				contentXML = "";
				break;

			default:
				getLogger().warn(
						"WMChange operation ignored in WMLogger: "
								+ _wmc.operation);
				break;
			}
			logger.debug("<WM_OP><WMC>"+wmcXML+"</WMC><CONTENT>" + contentXML+"</CONTENT></WM_OP>");

		} catch(CASTException e) {
			logger.warn("CASTException caught: ", e);
		}
		} 
	}

}
