/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/**
 * 
 */
package balt.clients;

import java.net.UnknownHostException;
import java.util.*;

import org.omg.CORBA.ORB;
import org.omg.CosNaming.*;
import org.omg.CosNaming.NamingContextPackage.*;

import balt.corba.autogen.FrameworkBasics.*;
import balt.corba.data.RemoteDataTranslator;
import balt.corba.impl.FrameworkProcessManagerException;
import balt.corba.impl.FrameworkProcessManagerServer;
import balt.core.connectors.LocalConnectionFactory;
import balt.core.data.BALTType;
import balt.management.FrameworkUtils;

/**
 * A bunch of static methods useful for balt clients.
 * 
 * @author nah
 */
public class ClientUtils {

	/**
	 * Method that actually does the work of browsing the namespace.
	 * 
	 * @param _orb
	 *            The CORBA orb.
	 * @param _nc
	 *            The naming context to browse.
	 * @param _indent
	 *            String used to indent the output.
	 * @throws NotFound
	 * @throws CannotProceed
	 * @throws InvalidName
	 */
	private static void browseNaming(ORB _orb, NamingContext _nc, String _indent)
			throws NotFound, CannotProceed, InvalidName {
		BindingListHolder bl = new BindingListHolder();
		BindingIteratorHolder blIt = new BindingIteratorHolder();
		_nc.list(1000, bl, blIt);
		Binding bindings[] = bl.value;
		if (bindings.length == 0)
			return;

		for (int i = 0; i < bindings.length; i++) {

			// get the object reference for each binding
			int lastIx = bindings[i].binding_name.length - 1;

			// check to see if this is a naming context
			if (bindings[i].binding_type == BindingType.ncontext) {
				System.out.println(_indent + "Context: "
						+ bindings[i].binding_name[lastIx].id);

				NameComponent nc2 = new NameComponent(
						bindings[i].binding_name[lastIx].id,
						bindings[i].binding_name[lastIx].kind);
				NameComponent[] name2 = {nc2};
				NamingContext ncContext = NamingContextHelper.narrow(_nc
						.resolve(name2));
				browseNaming(_orb, ncContext, _indent + "\t");

			}
			else {
				System.out.println(_indent + "Object: "
						+ bindings[i].binding_name[lastIx].id);
			}
		}

	}

	/**
	 * Now gone back to PREFIX:host naming scheme. This method now parses out
	 * the host and determines whether it matches the input. Not pretty.
	 * Contexts may actually work better, although neither seem to work properly
	 * with IP addresses. ACTUALLY it seems like it's IP addresses in process
	 * names that are the problem...
	 * 
	 * @param _nc
	 * @param _hostname
	 * @return
	 * @throws FrameworkProcessManagerException
	 */
	private static FrameworkProcessManager getProcessManager(
			NamingContextExt _nc, String _hostname)
			throws FrameworkProcessManagerException {

		BindingListHolder bl = new BindingListHolder();
		BindingIteratorHolder blIt = new BindingIteratorHolder();
		_nc.list(1000, bl, blIt);
		Binding bindings[] = bl.value;

		if (bindings.length == 0) {
			throw new FrameworkProcessManagerException(
					"Cannot get handle on FrameworkProcessManager on machine \""
							+ _hostname
							+ "\". Is there a process running on that machine?");
		}

		String objName, serverName;
		for (int i = 0; i < bindings.length; i++) {

			// get the object reference for each binding
			int lastIx = bindings[i].binding_name.length - 1;

			// check to see if this is a naming context
			if (bindings[i].binding_type == BindingType.nobject) {

				objName = bindings[i].binding_name[lastIx].id;

				// System.out.println("Obj: " + objName);

				if (objName
						.startsWith(FrameworkProcessManagerServer.SERVER_PREFIX)) {

					serverName = objName
							.substring(FrameworkProcessManagerServer.SERVER_PREFIX
									.length());

					try {

						// System.out.println("comparing");
						// System.out.println(_hostname);
						// System.out.println(serverName);

						if (FrameworkUtils.areSameHost(_hostname, serverName)) {

							NameComponent nc = new NameComponent(
									bindings[i].binding_name[lastIx].id,
									bindings[i].binding_name[lastIx].kind);
							NameComponent[] name = {nc};

							return FrameworkProcessManagerHelper.narrow(_nc
									.resolve(name));

						}
					}
					catch (UnknownHostException e) {
						throw new FrameworkProcessManagerException(
								"Cannot reach machine \"" + _hostname + "\"", e);
					}
					catch (Exception e) {
						throw new FrameworkProcessManagerException(
								"Cannot get handle on FrameworkProcessManager on machine \""
										+ _hostname
										+ "\". Is there a process running on that machine?",
								e);
					}
				}
			}
		}

		throw new FrameworkProcessManagerException(
				"No matching naming context on machine \"" + _hostname
						+ "\". Is there a process running on that machine?");
	}

	public static boolean addObjectDatatype(Class<?> _dataClass,
			Class<?> _pushRecvClass, Class<?> _pullRecvClass) {
		String _dataType = BALTType.typeName(_dataClass);
		if (!LocalConnectionFactory.supportsClass(_dataType)) {
			// System.out.println("Adding " + _dataType
			// + " datatype to balt.");

			// registers datatype with connection factory
			LocalConnectionFactory.addDatatype(_dataType, _dataClass,
					_pushRecvClass, _pullRecvClass);

			// registers datatype with translator for cross
			// language/machine support
			RemoteDataTranslator.addObjectTranslator(_dataClass);
			return true;
		}

		return false;
	}

	public static boolean addLocalObjectDatatype(Class<?> _dataClass,
			Class<?> _pushRecvClass, Class<?> _pullRecvClass) {

		String _dataType = BALTType.typeName(_dataClass);

		if (!LocalConnectionFactory.supportsClass(_dataType)) {
			// System.out.println("Adding " + _dataType
			// + " datatype to balt.");

			// registers datatype with connection factory
			LocalConnectionFactory.addDatatype(_dataType, _dataClass,
					_pushRecvClass, _pullRecvClass);

			return true;
		}

		return false;
	}

	public static boolean addSequenceDatatype(Class<?> _dataClass,
			Class<?> _helperClass,

			Class<?> _pushConnClass, Class<?> _pullConnClass) {
		String _dataType = BALTType.typeName(_dataClass);
		if (!LocalConnectionFactory.supportsClass(_dataType)) {
			// System.out.println("Adding " + _dataType
			// + " datatype to balt.");

			// registers datatype with connection factory
			LocalConnectionFactory.addDatatype(_dataType, _dataClass,
					_pushConnClass, _pullConnClass);

			// registers datatype with translator for cross
			// language/machine support
			RemoteDataTranslator
					.addSequenceTranslator(_dataClass, _helperClass);
			return true;
		}

		return false;
	}

	/**
	 * Print out the contents of the naming service.
	 * 
	 * @param _orb
	 *            The CORBA orb.
	 * @param _nc
	 *            The naming context to browse.
	 * @throws NotFound
	 * @throws CannotProceed
	 * @throws InvalidName
	 */
	public static void browseNaming(ORB _orb, NamingContext _nc)
			throws NotFound, CannotProceed, InvalidName {
		browseNaming(_orb, _nc, "");
	}

	/**
	 * Get a list of the hostnames used in the connections.
	 * 
	 * @param _connections
	 *            A list of process connection descriptions.
	 * @return A list of the unique hostnames from the descriptions.
	 * @throws FrameworkProcessManagerException
	 */
	public static String[] getFrameworkHostnames(
			ProcessConnection[] _connections)
			throws FrameworkProcessManagerException {
		ArrayList<String> hostnames = new ArrayList<String>();

		String hostname;

		System.out.print("Process Server: BALT hosts:");

		for (int i = 0; i < _connections.length; i++) {

			for (int j = 0; j < _connections[i].m_senders.length; j++) {

				hostname = _connections[i].m_senders[j].m_hostName;

				if (newHost(hostnames, hostname)) {
					hostnames.add(hostname);
					System.out.print(" " + hostname);
				}
			}

			for (int j = 0; j < _connections[i].m_receivers.length; j++) {

				hostname = _connections[i].m_receivers[j].m_hostName;

				if (newHost(hostnames, hostname)) {
					hostnames.add(hostname);
					System.out.print(" " + hostname);
				}
			}
		}

		System.out.println();

		if (hostnames.size() > 1 && hostnames.contains("localhost")) {
			throw new FrameworkProcessManagerException(
					"If you are using more than one host, do not use \"localhost\" in your configuration files. Hosts for this machine: "
							+ hostnames);
		}

		String[] hostnamesAsArray = new String[hostnames.size()];
		hostnames.toArray(hostnamesAsArray);
		return hostnamesAsArray;
	}

	/**
	 * @param hostnames
	 * @param hostname
	 * @return
	 * @throws FrameworkProcessManagerException
	 */
	private static boolean newHost(ArrayList<String> hostnames, String hostname)
			throws FrameworkProcessManagerException {

		for (String host : hostnames) {
			try {
				if (FrameworkUtils.areSameHost(host, hostname)) {
					return false;
				}
			}
			catch (UnknownHostException e) {
				throw new FrameworkProcessManagerException(
						"hostname lookup failure", e);
			}
		}
		return true;
	}

	/**
	 * Gets the FrameworkProcessManagers for the necessary hosts being used by
	 * the Framework.
	 * 
	 * @param _orb
	 *            The CORBA ORB.
	 * @param _hostnames
	 *            A list of hostnames to contact.
	 * @return A list of FrameworkProcessManager objects. One for each input
	 *         hostname.
	 * @throws org.omg.CORBA.ORBPackage.InvalidName
	 * @throws NotFound
	 * @throws CannotProceed
	 * @throws InvalidName
	 * @throws org.omg.CORBA.ORBPackage.InvalidName
	 * @throws FrameworkProcessManagerException
	 */
	public static FrameworkProcessManager[] getProcessManagers(ORB _orb,
			String[] _hostnames) throws FrameworkProcessManagerException {

		FrameworkProcessManager[] managers = new FrameworkProcessManager[_hostnames.length];

		// first task is to find the object to connect to
		// get the root naming context
		org.omg.CORBA.Object objRef;
		try {
			objRef = _orb.resolve_initial_references("NameService");
		}
		catch (org.omg.CORBA.ORBPackage.InvalidName e) {
			throw new FrameworkProcessManagerException(
					"Unable to access the NameService", e);
		}

		// Use NamingContextExt instead of NamingContext. This is
		// part of the Interoperable naming Service.
		NamingContextExt ncRef = NamingContextExtHelper.narrow(objRef);

		for (int i = 0; i < _hostnames.length; i++) {
			managers[i] = getProcessManager(ncRef, _hostnames[i]);
		}

		return managers;
	}

	/**
	 * Create a new ProcessDescription object. As this is a CORBA struct all
	 * fields must be filled in otherwise a null-pointer error is thrown when
	 * the object is written.
	 * 
	 * @param _name
	 *            Name of process.
	 * @param _class
	 *            Class of process.
	 * @param _lang
	 *            Language of process
	 * @param _host
	 *            Host of process.
	 * @return The new ProcessDescription.
	 */
	public static ProcessDescription newProcessDescription(String _name,
			String _class, ProcessLanguage _lang, String _host) {
		ProcessDescription procDesc = new ProcessDescription();
		procDesc.m_processName = _name;
		// procDesc.m_processName = _name + " on " + _host;
		procDesc.m_className = _class;
		procDesc.m_language = _lang;
		procDesc.m_hostName = _host;
		procDesc.m_configuration = new ProcessConfigurationMap(new String[0],
				new String[0]);
		return procDesc;
	}

	public static ProcessDescription newProcessDescription(String _name,
			String _class, ProcessLanguage _lang, String _host,
			Properties _config) {

		ProcessDescription procDesc = new ProcessDescription();
		procDesc.m_processName = _name;
		// procDesc.m_processName = _name + " on " + _host;
		procDesc.m_className = _class;
		procDesc.m_language = _lang;
		procDesc.m_hostName = _host;

		// System.out.println("nPD: " +
		// _config.getProperty("org.cognitivesystem.caat.subarchID"));
		// _config.keys()

		Enumeration<?> keysEnum = _config.propertyNames();

		ArrayList<String> keysList = new ArrayList<String>();
		while (keysEnum.hasMoreElements()) {
			keysList.add((String) keysEnum.nextElement());
		}

		String[] keys = new String[keysList.size()];
		String[] values = new String[keysList.size()];

		for (int i = 0; i < keysList.size(); i++) {
			keys[i] = keysList.get(i);
			values[i] = _config.getProperty((String) keys[i]);
			// System.out.println("newPD: " + keys[i]);
			// System.out.println("newPD: " + values[i]);

		}

		procDesc.m_configuration = new ProcessConfigurationMap(keys, values);

		// System.out.println("ClientUtils.newProcessDescription()");
		// _config.list(System.out);

		return procDesc;
	}

	public static ProcessDescription newProcessDescription(String _name,
			String _class, ProcessLanguage _lang, String _host, String _config) {
		ProcessDescription procDesc = new ProcessDescription();
		procDesc.m_processName = _name;
		// procDesc.m_processName = _name + " on " + _host;
		procDesc.m_className = _class;
		procDesc.m_language = _lang;
		procDesc.m_hostName = _host;
		procDesc.m_configuration = new ProcessConfigurationMap(
				new String[]{_config}, new String[]{_config});
		return procDesc;
	}

	/**
	 * @param namingHost
	 * @param namingPort
	 * @param connections
	 * @throws FrameworkProcessManagerException
	 * @throws InterruptedException
	 */
	public static void runFramework(String namingHost, String namingPort,
			ProcessConnection[] connections, long _runTime)
			throws FrameworkProcessManagerException {

		try {

			System.out.println("Process Server: Naming host: " + namingHost);

			// Store input properties
			Properties props = new Properties();
			props.put("org.omg.CORBA.ORBInitialPort", namingPort);
			props.put("org.omg.CORBA.ORBInitialHost", namingHost);
			// initialise orb
			ORB orb = ORB.init(new String[]{}, props);

			// get a list of the hostnames used in the connections
			String[] hostnames = getFrameworkHostnames(connections);

			// get the process managers for these hosts
			FrameworkProcessManager[] managers = getProcessManagers(orb,
					hostnames);

			// create the graph processes
			for (int i = 0; i < managers.length; i++) {
				managers[i].createGraph(connections);
			}

			// create any remote connection objects
			for (int i = 0; i < managers.length; i++) {
				managers[i].createRemoteConnections();
			}

			// sleep to give all the connections a chance to

			// start up (unnecessary?)
			Thread.sleep(1000);

			// connect all the objects locally and remotely
			for (int i = 0; i < managers.length; i++) {
				managers[i].connectGraph();
			}

			// sync time across servers as best we can
			for (int i = 0; i < managers.length; i++) {
				managers[i].synchroniseWatches();
			}

			// start the processes 
			for (int i = 0; i < managers.length; i++) {
				managers[i].startGraph();
			}

			// exectute the processes' run loops
			for (int i = 0; i < managers.length; i++) {
				managers[i].runGraph();
			}

			// if given a run time... if not then ctrl-c will have to do
			// ;)
			if (_runTime > 0) {
				// let the processes do something for a while
				try {
					Thread.sleep(_runTime);
				}
				catch (InterruptedException e) {
					e.printStackTrace();
				}

				// kill off the processes
				for (int i = 0; i < managers.length; i++) {
					managers[i].stopGraph();
				}
				// System.out.println("shutting down orb");
				orb.shutdown(true);
				System.exit(0);
				// System.out.println("done");

			}

		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}
		catch (Throwable t) {
			throw new FrameworkProcessManagerException("Failure during run", t);
		}

	}

	/**
	 * @param namingHost
	 * @param namingPort
	 * @param connections
	 * @throws FrameworkProcessManagerException
	 * @throws InterruptedException
	 */
	public static void stopFramework(String namingHost, String namingPort,
			ProcessConnection[] connections)
			throws FrameworkProcessManagerException {

		try {

			System.out.println("Process Server: Naming host: " + namingHost);

			// Store input properties
			Properties props = new Properties();
			props.put("org.omg.CORBA.ORBInitialPort", namingPort);
			props.put("org.omg.CORBA.ORBInitialHost", namingHost);
			// initialise orb
			ORB orb = ORB.init(new String[]{}, props);

			// get a list of the hostnames used in the connections
			String[] hostnames = getFrameworkHostnames(connections);

			// get the process managers for these hosts
			FrameworkProcessManager[] managers = getProcessManagers(orb,
					hostnames);

			// kill off the processes
			for (int i = 0; i < managers.length; i++) {
				managers[i].stopGraph();
			}
			// System.out.println("shutting down orb");
			orb.shutdown(true);
			System.exit(0);
			// System.out.println("done");

		}
		catch (Throwable t) {
			throw new FrameworkProcessManagerException("Failure during run", t);
		}

	}

	// public static void idlj() {
	// org.cognitivesystems.idl.toJavaPortable.Compile comp = Factories
	// }
}
