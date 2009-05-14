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

import org.omg.CORBA.ORBPackage.InvalidName;
import org.omg.CosNaming.NamingContextPackage.CannotProceed;
import org.omg.CosNaming.NamingContextPackage.NotFound;
import org.omg.PortableServer.POAManagerPackage.AdapterInactive;

import balt.clients.ClientUtils;
import balt.corba.autogen.FrameworkBasics.ProcessConnection;
import balt.corba.impl.FrameworkProcessManagerServer;
import cast.configuration.ArchitectureConfigurationException;
import cast.configuration.CASTConfigParser;
import cast.configuration.CASTConnectionConfiguration;

/**
 * @author nah
 */
public class StopServer extends Thread {

	private String m_namingHost;

	private String m_namingPort;

	private String[] m_args;

	private boolean m_debug;

	private String m_serverName;

	public static final String m_os = System.getProperty("os.name");

	public static final String MAC_OS_X = "Mac OS X";

	public StopServer(String _namingHost, String _namingPort, boolean _debug,
			String _serverName, String[] _args) {
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
		String namingHost = null;
		String namingPort = "1050";
		String configFile = null;

		long runTime = 0;
		boolean useGUI = false;
		boolean debug = false;
		String serverName = null;

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

		// now see if we're the configuration server or not
		if (configFile != null) {
			try {
				CASTConnectionConfiguration ccc = CASTConfigParser
						.parseConfigFile(configFile);

				ProcessConnection[] extras = CASTConfigParser
						.parseExtras(configFile);

				ArrayList<ProcessConnection> connections = ccc.getConnections();

				for (ProcessConnection extrs : extras) {
					connections.add(extrs);
				}

				ProcessConnection[] conns = (ProcessConnection[]) connections
						.toArray(new ProcessConnection[connections.size()]);

				ClientUtils.stopFramework(namingHost, namingPort, conns);

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
	 * 
	 */
	private static void showArgs() {
		System.err
				.println("CAATProcessServer arguments:\n\t -h naming host REQUIRED\n\t -f config file OPTIONAL\n\t -r millisecond run time OPTIONAL\\n\\t -d debug mode OPTIONAL");
	}

}
