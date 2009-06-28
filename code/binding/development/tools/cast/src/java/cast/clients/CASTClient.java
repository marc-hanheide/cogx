package cast.clients;

import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;

import Ice.Application;
import Ice.Communicator;
import Ice.Identity;
import Ice.ObjectAdapter;
import Ice.ObjectNotExistException;
import Ice.ObjectPrx;
import cast.ComponentCreationException;
import cast.cdl.CASTRELEASESTRING;
import cast.cdl.CPPSERVERPORT;
import cast.cdl.ComponentDescription;
import cast.cdl.ComponentLanguage;
import cast.cdl.JAVACLIENTSERVERPORT;
import cast.cdl.JAVASERVERPORT;
import cast.configuration.ArchitectureConfigurationException;
import cast.configuration.CASTConfigParser;
import cast.configuration.SubarchitectureConfiguration;
import cast.configuration.SubarchitectureProxies;
import cast.core.CASTUtils;
import cast.interfaces.CASTComponentPrx;
import cast.interfaces.ComponentFactoryPrx;
import cast.interfaces.ComponentManagerPrx;
import cast.interfaces.ComponentManagerPrxHelper;
import cast.interfaces.ManagedComponentPrx;
import cast.interfaces.TaskManagerPrx;
import cast.interfaces.TimeServerPrx;
import cast.interfaces.TimeServerPrxHelper;
import cast.interfaces.WorkingMemoryAttachedComponentPrx;
import cast.interfaces.WorkingMemoryPrx;
import cast.server.CASTComponentManager;

public class CASTClient extends Application {

	// public CASTClient() {
	// //don't handle... means we have to clean up
	// super(SignalPolicy.NoSignalHandling);
	//		
	// }

	/**
	 * 
	 */
	private static void showArgs() {
		System.err
				.println("CASTProcessServer arguments: -f config file REQUIRED");
	}

	private ArrayList<CASTComponentPrx> m_components;
	private HashMap<String, ComponentDescription> m_componentDescriptions;

	@Override
	public int run(String[] args) {

		System.out.println("CoSy Architecture Schema Toolkit. Release: "
				+ CASTRELEASESTRING.value);

		String configFile = null;

		try {
			for (int i = 0; i < args.length; i++) {
				if (args[i].equals("-f")) {
					configFile = args[i + 1];
				}
			}
		} catch (ArrayIndexOutOfBoundsException e) {
			showArgs();
			return 1;
		}

		if (configFile == null) {
			showArgs();
			return 1;
		}

		// now see if we're the configuration server or not
		// if (configFile != null) {
		try {
			CASTConfigParser.parseConfigFile(configFile);

			// very bad design... reusing old crap

			// HashSet<String> hosts =
			// getHosts(CASTConfigParser.m_subarchitectures);
			//
			// System.out.println("CAST hosts: " + hosts);
			//
			// List<ComponentServerPrx> servers = new
			// ArrayList<ComponentServerPrx>();

			//
			// for (String host : hosts) {
			// servers.add(getCPPServer(host));
			// }

			ComponentManagerPrx manager = getComponentManager();
			HashMap<String, TimeServerPrx> timeServers = getTimeServers();

			// reset all timeservers
			// TODO make this more principled
			for (TimeServerPrx ts : timeServers.values()) {
				ts.reset();
			}

			// very bad design... reusing old crap

			// store descrs for later
			storeDescriptions();

			ArrayList<SubarchitectureProxies> subarchs = getProxies(
					CASTConfigParser.m_subarchitectures, manager, timeServers);

			m_components = new ArrayList<CASTComponentPrx>();

			// join up the dots
			for (SubarchitectureProxies subarch : subarchs) {

				// connect up components within subarchs
				connectSubarchitecture(subarch, m_components);

				// connect across subarchs
				for (SubarchitectureProxies otherSubarch : subarchs) {
					if (otherSubarch != subarch) {
						subarch.getWorkingMemory().setWorkingMemory(
								otherSubarch.getWorkingMemory(),
								otherSubarch.getID());
					}
				}
			}

			// now get the extra, non-sa components
			assert (CASTConfigParser.m_extras != null);
			for (ComponentDescription cd : CASTConfigParser.m_extras) {
				int compPort = CPPSERVERPORT.value;
				if (cd.language == ComponentLanguage.JAVA) {
					compPort = JAVASERVERPORT.value;
				}
				CASTComponentPrx prx = CASTUtils.createCASTComponent(
						new Identity(cd.componentName, cd.className),
						communicator(), cd.hostName, compPort);
				prx.setComponentManager(manager);

				TimeServerPrx ts = timeServers.get(cd.hostName);
				assert (ts != null);
				prx.setTimeServer(ts);
				prx.configure(cd.configuration);
				m_components.add(prx);
			}

			startComponents();

			runComponents();

			// install a signal handler
			// Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
			// public void run() {
			// stopComponents();
			// }
			// }));

			setInterruptHook(new Thread(new Runnable() {
				public void run() {
					stopComponents();
					destroyComponents();
				}
			}));

			communicator().waitForShutdown();

			return 0;

		} catch (ArchitectureConfigurationException e) {
			e.printStackTrace();
		}
		// catch (InterruptedException e) {
		// e.printStackTrace();
		// return 1;
		// }
		catch (ComponentCreationException e) {
			e.printStackTrace();
		}

		return 1;

	}

	/**
	 * 
	 */
	private void storeDescriptions() {
		m_componentDescriptions = new HashMap<String, ComponentDescription>();
		for (SubarchitectureConfiguration sa : CASTConfigParser.m_subarchitectures) {
			ArrayList<ComponentDescription> descriptions = sa
					.getDescriptions();
			for (ComponentDescription cd : descriptions) {
				m_componentDescriptions.put(cd.componentName, cd);
			}
		}
		for (ComponentDescription cd : CASTConfigParser.m_extras) {
			m_componentDescriptions.put(cd.componentName, cd);
		}
	}

	private HashMap<String, TimeServerPrx> getTimeServers() {
		return getTimeServers(getHosts());
	}

	private HashMap<String, TimeServerPrx> getTimeServers(HashSet<String> _hosts) {

		HashMap<String, TimeServerPrx> servers = new HashMap<String, TimeServerPrx>();
		ArrayList<String> leftovers = new ArrayList<String>(_hosts.size());

		for (String host : _hosts) {
			try {
				TimeServerPrx ts = getTimeServer(host, CPPSERVERPORT.value);
				servers.put(host, ts);
			} catch (RuntimeException e) {
				e.printStackTrace();
				// if one not found, store for later
				leftovers.add(host);
			}
		}

		if (servers.isEmpty()) {
			throw new RuntimeException(
					"Unable to find any time servers anywhere. Try running the C++ component server somewhere.");
		}

		TimeServerPrx fallback = servers.values().iterator().next();

		for (String host : leftovers) {
			servers.put(host, fallback);
		}

		return servers;
	}

	/**
	 * 
	 * Try possible combinations of servers to get the component manager for the
	 * whole system. This is currently a bit pointless as it doesn't do
	 * anything.
	 * 
	 * @return
	 */
	private ComponentManagerPrx getComponentManager() {
		String javaHost = getAJavaHost(CASTConfigParser.m_subarchitectures);

		//
		ComponentManagerPrx manager;

		if (javaHost == null) {

			// try on any host we have instead, in case there are java
			// servers running there anyway
			String anyHost = null;
			if (!CASTConfigParser.m_subarchitectures.isEmpty()) {
				anyHost = CASTConfigParser.m_subarchitectures.get(0)
						.getWorkingMemoryConfig().hostName;
			} else if (!CASTConfigParser.m_extras.isEmpty()) {
				anyHost = CASTConfigParser.m_extras.get(0).hostName;
			}

			if (anyHost == null) {
				throw new RuntimeException(
						"Your configuration file contains no components");
			}

			try {
				// try to get it there
				manager = getComponentManager(anyHost, JAVASERVERPORT.value);
			} catch (ObjectNotExistException e) {
				// if fail, then run it ourselves
				setupServer();
				manager = getComponentManager(getExternalInterface(),
						JAVACLIENTSERVERPORT.value);
			}
		} else {
			manager = getComponentManager(javaHost, JAVASERVERPORT.value);
		}
		assert (manager != null);
		return manager;
	}

	private void runComponents() {
		// System.out.println("running components");
		int max = m_components.size();
		int count = 1;

		for (CASTComponentPrx cmp : m_components) {

			// only for used during development
			System.out.println("[running [" + count++ + "/" + max + "]: "
					+ cmp.getID() + "]");

			// System.out.println(cmp.getID());

			cmp.run();
		}
	}

	private void startComponents() {
		// System.out.println("starting components");

		int max = m_components.size();
		int count = 1;
		for (CASTComponentPrx cmp : m_components) {

			// only for used during development
			System.out.println("[starting [" + count++ + "/" + max + "]: "
					+ cmp.getID() + "]");
			cmp.start();
		}
	}

	private void stopComponents() {
		int max = m_components.size();
		int count = 1;

		// System.out.println("stopping components");
		for (CASTComponentPrx cmp : m_components) {
			// only for used during development
			System.out.println("[stopping [" + count++ + "/" + max + "]: "
					+ cmp.getID() + "]");
			cmp.stop();
		}

	}

	private void destroyComponents() {
		int max = m_components.size();
		int count = 1;

		// System.out.println("stopping components");
		for (CASTComponentPrx cmp : m_components) {
			// only for used during development
			String id = cmp.getID();
			System.out.println("[destroying [" + count++ + "/" + max + "]: "
					+ id + "]");
//			ComponentFactoryPrx factory = CASTUtils
//					.getComponentFactoryForID(id);
//			
//			ComponentDescription cd = m_componentDescriptions.get(id);
//			assert(cd != null);
//			factory.remove(id, cd.className);
			cmp.destroy();
		}

	}

	private String getExternalInterface() {

		try {
			Enumeration<NetworkInterface> networkInterfaces = NetworkInterface
					.getNetworkInterfaces();

			while (networkInterfaces.hasMoreElements()) {
				NetworkInterface netface = (NetworkInterface) networkInterfaces
						.nextElement();

				System.out.println("Net interface: " + netface.getName());

				Enumeration<?> e2 = netface.getInetAddresses();

				while (e2.hasMoreElements()) {
					InetAddress ip = (InetAddress) e2.nextElement();

					if (ip instanceof Inet4Address) {

						System.out.println("-------");
						// System.out.println("link: " +
						// ip.isLinkLocalAddress());
						// System.out.println("any: " + ip.isAnyLocalAddress());
						// System.out.println("loop: " +
						// ip.isLoopbackAddress());
						// System.out.println(ip.getCanonicalHostName());
						System.out.println(ip.getHostAddress());
						// //
						System.out.println("-------");

						if (!ip.isLinkLocalAddress() && !ip.isLoopbackAddress()) {
							return ip.getHostAddress();
						}

					}
				}
			}
		} catch (SocketException e) {
			e.printStackTrace();
		}

		System.out
				.println("Unable to find a suitable network address to connect to for component manager, using localhost instead");
		return "localhost";
	}

	private TimeServerPrx getTimeServer(String _host, int _port) {

		Identity id = new Identity("TimeServer", "TimeServer");
		ObjectPrx base = communicator().stringToProxy(
				communicator().identityToString(id) + ":default -h " + _host
						+ " -p " + _port);
		if (base == null) {
			throw new RuntimeException(
					"Cannot create proxy to CPP TimeServer on " + _host);
		}

		TimeServerPrx prx = TimeServerPrxHelper.checkedCast(base);
		assert (prx != null);
		return prx;
	}

	private ComponentManagerPrx getComponentManager(String _host, int _port) {

		// Identity id = new Identity("comp.man",
		// _ComponentManagerDisp.ice_staticId());
		Identity id = new Identity("comp.man", CASTComponentManager.class
				.getCanonicalName());

		ObjectPrx base = communicator().stringToProxy(
				communicator().identityToString(id) + ":default -h " + _host
						+ " -p " + _port);

		if (base == null) {
			throw new RuntimeException(
					"Cannot create proxy to CPP ComponentServer on " + _host);
		}

		ComponentManagerPrx prx = ComponentManagerPrxHelper.checkedCast(base);
		assert (prx != null);
		return prx;
	}

	private void setupServer() {
		// System.out.println("CASTClient.setupServer()");
		Communicator ic = communicator();
		ObjectAdapter adapter = ic.createObjectAdapterWithEndpoints(
				"ComponentServer2", "default -p " + JAVACLIENTSERVERPORT.value);

		Identity manid = new Identity("comp.man", CASTComponentManager.class
				.getCanonicalName());

		adapter.add(new CASTComponentManager(), manid);

		adapter.activate();
		// System.out.println("CASTClient.setupServer() DONE");

		// try {
		// Thread.sleep(2000);
		// } catch (InterruptedException e) {
		// e.printStackTrace();
		// }

	}

	// private ComponentServerPrx getCPPServer(String _host) {
	// Identity id = new Identity("cpp.server", ComponentServer.class
	// .getCanonicalName());
	// ObjectPrx base = communicator().stringToProxy(
	// communicator().identityToString(id) + ":default -h " + _host
	// + " -p " + CPPSERVERPORT.value);
	//
	// if (base == null) {
	// throw new RuntimeException(
	// "Cannot create proxy to CPP ComponentServer on " + _host);
	// }
	// return ComponentServerPrxHelper.checkedCast(base);
	// }

	private HashSet<String> getHosts() {
		HashSet<String> hosts = getHosts(CASTConfigParser.m_subarchitectures);

		for (ComponentDescription cd : CASTConfigParser.m_extras) {
			hosts.add(cd.hostName);
		}

		return hosts;
	}

	private HashSet<String> getHosts(
			ArrayList<SubarchitectureConfiguration> _m_subarchitectures) {
		HashSet<String> hosts = new HashSet<String>();
		for (SubarchitectureConfiguration sa : _m_subarchitectures) {
			hosts.addAll(sa.getHosts());
		}
		return hosts;
	}

	private void connectSubarchitecture(SubarchitectureProxies _subarch,
			List<CASTComponentPrx> _components) {
		// System.out.println("CASTClient.connectSubarchitecture()");

		WorkingMemoryPrx wm = _subarch.getWorkingMemory();
		TaskManagerPrx tm = _subarch.getTaskManager();

		_components.add(wm);
		_components.add(tm);

		for (ManagedComponentPrx prx : _subarch.getManagedComponents()) {

			assert (prx != null);
			assert (wm != null);
			tm.addManagedComponent(prx);
			prx.setWorkingMemory(wm);
			wm.addReader(prx);
			_components.add(prx);
		}

		for (WorkingMemoryAttachedComponentPrx prx : _subarch
				.getUnmanagedComponents()) {

			assert (prx != null);
			assert (wm != null);

			prx.setWorkingMemory(wm);
			_components.add(prx);
		}
	}

	private String getAJavaHost(
			ArrayList<SubarchitectureConfiguration> _m_subarchitectures) {
		for (SubarchitectureConfiguration subarch : _m_subarchitectures) {
			if (subarch.getAJavaServer() != null) {
				return subarch.getAJavaServer();
			}
		}
		return null;
	}

	private ArrayList<SubarchitectureProxies> getProxies(
			ArrayList<SubarchitectureConfiguration> _m_subarchitectures,
			ComponentManagerPrx _man, Map<String, TimeServerPrx> _timeServers)
			throws ComponentCreationException {
		ArrayList<SubarchitectureProxies> proxies = new ArrayList<SubarchitectureProxies>();
		for (SubarchitectureConfiguration subarch : _m_subarchitectures) {
			proxies.add(new SubarchitectureProxies(subarch, _man, _timeServers,
					communicator()));
		}
		return proxies;
	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		// System.out.println("CASTClient.main()");
		CASTClient app = new CASTClient();
		int status = app.main("CASTClient", args);
		System.exit(status);
	}

}
