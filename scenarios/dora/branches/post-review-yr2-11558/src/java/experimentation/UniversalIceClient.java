package experimentation;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import org.apache.log4j.BasicConfigurator;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import cast.CASTException;
import cast.cdl.ComponentLanguage;
import cast.core.CASTUtils;

/**
 * universal cast client to invoke method on server example call: java -cp"/usr/share/java/log4j-1.2.jar:/usr/share/java/Ice.jar:/usr/local/share/java/cast.jar:./output/classes"
 * experimentation.UniversalIceClient place.manager localhost
 * FrontierInterface.PlaceInterface getCurrentPlace
 * 
 * @author marc
 * 
 */
public class UniversalIceClient {

	String serverName;
	String serverHost;
	String serverClassName;
	String methodName;
	Ice.ObjectPrx server = null;
	Class<? extends Ice.Object> serverClass = null;
	Class<? extends Ice.ObjectPrx> serverPrxClass = null;
	Method methodToCall = null;

	Logger logger;

	@SuppressWarnings("unchecked")
	public void init() throws ClassNotFoundException, CASTException {
		logger.debug("trying to instantiate server classes for "
				+ serverClassName + " by reflection");
		serverClass = (Class<? extends Ice.Object>) Class
				.forName(serverClassName);
		serverPrxClass = (Class<? extends Ice.ObjectPrx>) Class
				.forName(serverClassName + "Prx");

		logger.debug("finding method " + methodName + "in server class "
				+ serverPrxClass.getSimpleName());
		Method[] methods = serverPrxClass.getMethods();
		for (Method m : methods) {
			if (m.getName().equals(methodName)) {
				methodToCall = m;
				logger.debug("found it: " + m);
				break;
			}
		}

		if (methodToCall == null) {
			throw new IllegalArgumentException("couldn't find method "
					+ methodName + "in class " + serverPrxClass.getSimpleName());
		}

		logger.debug("creating server proxy");
		try {
			server = CASTUtils.getCASTIceServer(serverName, serverHost,
					ComponentLanguage.CPP, serverClass, serverPrxClass);
		} catch (CASTException e) {
			server = CASTUtils.getCASTIceServer(serverName, serverHost,
					ComponentLanguage.JAVA, serverClass, serverPrxClass);

		}

	}

	public void call(String[] args) throws IllegalArgumentException,
			IllegalAccessException, InvocationTargetException {
		for (String a : args) {
			if (a.equals("null"))
				a = null;
		}
		logger.info("calling " + serverPrxClass.getSimpleName() + ":"
				+ methodToCall.getName() + " with " + args.length
				+ " arguments");
		Object answer = methodToCall.invoke(server, (Object[]) args);
		if (answer != null) {
			logger.info("call returned with object of type "
					+ answer.getClass().getName() + " == " + answer.toString());
		}
	}

	/**
	 * @param serverName
	 * @param serverObjectName
	 * @param methodName
	 */
	public UniversalIceClient(String serverName, String serverHost,
			String serverObjectName, String methodName) {
		super();
		logger = Logger.getLogger(UniversalIceClient.class);
		this.serverName = serverName;
		this.serverHost = serverHost;
		this.serverClassName = serverObjectName;
		this.methodName = methodName;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		BasicConfigurator.configure();
		Logger.getRootLogger().setLevel(Level.DEBUG);
		if (args.length >= 4) {
			String[] invokeArgs = new String[args.length - 4];
			for (int i = 4; i < args.length; i++) {
				invokeArgs[i - 4] = args[i];
			}
			UniversalIceClient client = new UniversalIceClient(args[0],
					args[1], args[2], args[3]);
			try {
				client.init();
				client.call(invokeArgs);
			} catch (CASTException e) {
				Logger.getLogger(UniversalIceClient.class).error("failed", e);
			} catch (ClassNotFoundException e) {
				Logger.getLogger(UniversalIceClient.class).error("failed", e);
			} catch (IllegalArgumentException e) {
				Logger.getLogger(UniversalIceClient.class).error("failed", e);
			} catch (IllegalAccessException e) {
				Logger.getLogger(UniversalIceClient.class).error("failed", e);
			} catch (InvocationTargetException e) {
				Logger.getLogger(UniversalIceClient.class).error("failed", e);
			} catch (RuntimeException e) {
				Logger.getLogger(UniversalIceClient.class).error("failed", e);
			} finally {
				System.exit(0);
			}

		} else {
			Logger
					.getLogger(UniversalIceClient.class)
					.info(
							"illegal usage; should be "
									+ UniversalIceClient.class.getSimpleName()
									+ " serverName serverHost serverObjectName methodName [args...]");
		}

	}

}
