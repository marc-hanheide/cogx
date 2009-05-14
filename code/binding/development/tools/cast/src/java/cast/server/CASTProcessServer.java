/*
 * CAST - The CoSy Architecture Schema Toolkit Copyright (C) 2006-2007
 * Nick Hawes This library is free software; you can redistribute it
 * and/or modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either version
 * 2.1 of the License, or (at your option) any later version. This
 * library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. You should have
 * received a copy of the GNU Lesser General Public License along with
 * this library; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * 
 */
package cast.server;

import java.util.ArrayList;
import java.util.HashMap;

import org.omg.CORBA.ORBPackage.InvalidName;
import org.omg.CosNaming.NamingContextPackage.CannotProceed;
import org.omg.CosNaming.NamingContextPackage.NotFound;
import org.omg.PortableServer.POAManagerPackage.AdapterInactive;

import balt.corba.autogen.FrameworkBasics.FrameworkConnectionType;
import balt.corba.autogen.FrameworkBasics.ProcessConfigurationMap;
import balt.corba.autogen.FrameworkBasics.ProcessConnection;
import balt.corba.autogen.FrameworkBasics.ProcessDescription;
import balt.corba.autogen.FrameworkBasics.ProcessLanguage;
import balt.corba.impl.FrameworkProcessManagerServer;
import cast.cdl.CAST_RELEASE_STRING;
import cast.cdl.guitypes.DrawBatch;
import cast.cdl.ui.ComponentStatus;
import cast.cdl.ui.TextOutput;
import cast.configuration.ArchitectureConfigurationException;
import cast.configuration.CASTConfigParser;
import cast.configuration.CASTConnectionConfiguration;
import cast.core.CASTUtils;

/**
 * @author nah
 */
public class CASTProcessServer extends Thread {

	private String m_namingHost;

	private String m_namingPort;

	private String[] m_args;

	private boolean m_debug;

	private String m_serverName;

	public static final String m_os = System.getProperty("os.name");

	public static final String MAC_OS_X = "Mac OS X";

	public CASTProcessServer(String _namingHost, String _namingPort,
			boolean _debug, String _serverName, String[] _args) {
		m_namingHost = _namingHost;
		m_namingPort = _namingPort;
		m_args = _args;
		m_debug = _debug;
		m_serverName = _serverName;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	@Override
	public void run() {
		try {
			FrameworkProcessManagerServer.launchProcessServer(m_args,
					m_namingHost, m_namingPort, m_debug, m_serverName);
		} catch (InvalidName e) {
			e.printStackTrace();
			System.exit(1);
		} catch (AdapterInactive e) {
			e.printStackTrace();
			System.exit(1);
		} catch (org.omg.CosNaming.NamingContextPackage.InvalidName e) {
			e.printStackTrace();
			System.exit(1);
		} catch (NotFound e) {
			e.printStackTrace();
			System.exit(1);
		} catch (CannotProceed e) {
			e.printStackTrace();
			System.exit(1);
		} catch (Throwable t) {
			t.printStackTrace();
			System.err.println("Make sure you have tnameserv running on "
					+ m_namingHost + " on port # " + m_namingPort);
			System.exit(1);
		}

	}

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		System.out.println("CoSy Architecture Schema Toolkit. Release: " + CAST_RELEASE_STRING.value);
		
		String namingHost = null;
		String namingPort = "1050";
		String configFile = null;

		long runTime = 0;
		boolean useGUI = false;
		boolean debug = false;
		String serverName = null;
		String guiHost = null;
		
		try {
			for (int i = 0; i < args.length; i++) {
				if (args[i].equals("-f")) {
					configFile = args[i + 1];
				} else if (args[i].equals("-h")) {
					namingHost = args[i + 1];
				} else if (args[i].equals("-d")) {
					debug = true;
				} else if (args[i].equals("-n")) {
					serverName = args[i + 1];
				} else if (args[i].equals("-r")) {
					try {
						runTime = Long.parseLong(args[i + 1]);
					} catch (NumberFormatException e) {
						e.printStackTrace();
						System.err.println("Incorrect runtime specification: "
								+ args[i + 1]);
					}
				} else if (args[i].equals("-g")) {
					useGUI = true;
					guiHost = args[i + 1];
				}

			}
		} catch (ArrayIndexOutOfBoundsException e) {
			showArgs();
			return;
		}

		if (namingHost == null) {
			showArgs();
			return;
		}

		// start server
		CASTProcessServer server = new CASTProcessServer(namingHost,
				namingPort, debug, serverName, args);
		server.start();

		// sleep a little to ensure it has started properly
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		// now see if we're the configuration server or not
		if (configFile != null) {
			try {
				CASTConnectionConfiguration ccc = CASTConfigParser
						.parseConfigFile(configFile);

				// ArrayList<ProcessConnection> conns =
				// ccc.getConnections();
				// for (ProcessConnection conn : conns) {
				// System.out.println("conn: " + conn);
				// }

				ProcessConnection[] extras = CASTConfigParser
						.parseExtras(configFile);

				if (useGUI) {

					if (m_os.equals(MAC_OS_X)) {
						System.out
								.println("Applying OpenGL hack for Mac OS X... have a nice day.");

						try {
							/*
							 * HACK: The OpenGL integration code from Apple
							 * loads and initialises the following class. If
							 * this initialisation happens within a CAAT process
							 * it just seems to hang. If it happens here then
							 * it's ok. As classes are only initialised once,
							 * this circumvents the problem.
							 */
							String className = "apple.awt.CHIViewEmbeddedFrame";
							Class.forName(className, true, Thread
									.currentThread().getContextClassLoader());
						} catch (Throwable e) {
							e.printStackTrace();
							System.exit(1);
						}
					}

					ArrayList<ProcessConnection> guiConnections = setupGUIConnections(ccc, guiHost);

					for (ProcessConnection connection : extras) {
						guiConnections.add(connection);
					}

					extras = new ProcessConnection[guiConnections.size()];
					guiConnections.toArray(extras);

				}

				CASTUtils.runCAST(namingHost, ccc, extras, runTime);

			} catch (ArchitectureConfigurationException e) {
				e.printStackTrace();
				System.exit(1);
			} catch (Throwable t) {
				t.printStackTrace();
				System.exit(1);
			}
		}

	}

	/**
	 * @param _ccc
	 * @return
	 * @throws ArchitectureConfigurationException
	 */
	private static ArrayList<ProcessConnection> setupGUIConnections(
			CASTConnectionConfiguration _ccc, String _guiHost)
			throws ArchitectureConfigurationException {
		ArrayList<ProcessConnection> connections = _ccc.getConnections();

		HashMap<String, ProcessDescription> processes = new HashMap<String, ProcessDescription>();

		for (ProcessConnection connection : connections) {
//			 System.out.println("guify?: " + connection.m_dataType);
//			// only GUIfy these connections
//			if (connection.m_dataType.equals(CASTUtils.typeName(WorkingMemoryChange[].class))
//					|| connection.m_dataType.equals(CASTUtils.typeName(WorkingMemoryEntry.class))
//					|| connection.m_dataType.equals(CASTUtils.typeName(CASTWorkingMemoryEntry.class))) {

				for (int i = 0; i < connection.m_senders.length; i++) {
					processes.put(connection.m_senders[i].m_processName,
							connection.m_senders[i]);
//					 	System.out.println("guifying : " + connection.m_senders[i].m_processName);
				}

				for (int i = 0; i < connection.m_receivers.length; i++) {
					processes.put(connection.m_receivers[i].m_processName,
							connection.m_receivers[i]);
//					System.out.println("guifying : " + connection.m_receivers[i].m_processName);
				}
//			}
		}

		System.out.println("Process Server: GUI host: " + _guiHost);

		ProcessDescription guiProcess = new ProcessDescription("cast.gui",
				"cast.ui.UnifiedGUI", ProcessLanguage.JAVA_PROCESS,
				_guiHost, new ProcessConfigurationMap(
						new String[] { CASTConfigParser.CONFIG_FILE_FLAG },
						new String[] { CASTConfigParser.getLastParse() }));

		ProcessDescription[] allProcs = new ProcessDescription[processes.size()];
		processes.values().toArray(allProcs);

		// for (ProcessDescription description : allProcs) {
		// System.out.println("allprocs: " + description);
		// }

		ArrayList<ProcessConnection> guiConnections = new ArrayList<ProcessConnection>();

		// push connections to gui
		guiConnections.add(new ProcessConnection(allProcs,
				new ProcessDescription[] { guiProcess },
				FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
						.typeName(TextOutput.class), "gui:text:push"));

		// guiConnections.add(new ProcessConnection(allProcs,
		// new ProcessDescription[] {
		// guiProcess
		// }, FrameworkConnectionType.PUSH_CONNECTION,
		// "ComponentEvent", "gui:event:push"));

		guiConnections.add(new ProcessConnection(allProcs,
				new ProcessDescription[] { guiProcess },
				FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
						.typeName(DrawBatch.class), "gui:draw:push"));

		int i = 0;
		for (ProcessDescription desc : allProcs) {

			guiConnections.add(new ProcessConnection(
					new ProcessDescription[] { guiProcess },
					new ProcessDescription[] { desc },
					FrameworkConnectionType.PULL_CONNECTION, CASTUtils
							.typeName(ComponentStatus.class),
					"gui:status:pull:" + i++));

			guiConnections
					.add(new ProcessConnection(
							new ProcessDescription[] { guiProcess },
							new ProcessDescription[] { desc },
							FrameworkConnectionType.PULL_CONNECTION, CASTUtils
									.typeName(DrawBatch.class),
							"gui:draw:pull:" + i++));

		}

		return guiConnections;

	}

	/**
	 * 
	 */
	private static void showArgs() {
		System.err
				.println("CAATProcessServer arguments:\n\t -h naming host REQUIRED\n\t -f config file OPTIONAL\n\t -r millisecond run time \n\t -g <hostname> run gui on hostname OPTIONAL\\n\\t -d debug mode OPTIONAL");
	}

}
