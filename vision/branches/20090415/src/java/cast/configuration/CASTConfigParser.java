/*
 * CAST - The CoSy Architecture Schema Toolkit
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
package cast.configuration;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.NoSuchElementException;

import cast.cdl.COMPONENTIDSKEY;
import cast.cdl.COMPONENTNUMBERKEY;
import cast.cdl.CONFIGFILEKEY;
import cast.cdl.ComponentDescription;
import cast.cdl.ComponentLanguage;
import cast.cdl.WMIDSKEY;

/**
 * An object that builds an architecture configuration from a config file.
 * 
 * @author nah
 */
public class CASTConfigParser {

	/**
	 * 
	 */
	public static final String CONFIG_FILE_FLAG = "--config-file";

	private static final String CPP_FLAG = "CPP";

	private static final String DD_FLAG = "DD";

	private static final String EXTRA_COMPONENT_HEADER = "COMPONENT";

	private static final String EXTRA_CONNECTION_HEADER = "CONNECTION";

	private static String EXTRA_CONNECTION_ID = "extra";

	private static final String GD_FLAG = "GD";

	private static final String UM_FLAG = "UM";

	private static final String MG_FLAG = "MG";

	private static final String JAVA_FLAG = "JAVA";

	private static ArchitectureConfiguration m_architecture;

	private static String m_defaultHost;

	private static int m_extraConnectionCount;

	private static ArrayList<ComponentDescription> m_extraDescriptions;

	public static ArrayList<SubarchitectureConfiguration> m_subarchitectures;

	private static HashSet<String> m_componentNames;

	private static HashSet<String> m_subarchNames;


	private static final String TM_FLAG = "TM";

	private static final String TRUE_VALUE = "true";

	private static final String WM_FLAG = "WM";

	public static final String ARCH_HEADER = "ARCHITECTURE";

	public static final String COMMENT_CHAR = "#";

	public static final String HOST_HEADER = "HOST";

	public static final String SUBARCH_HEADER = "SUBARCHITECTURE";

	private static String m_lastParse = "";

	private static String m_currentFile;
//
//	/**
//	 * @param _connection
//	 * @param _string
//	 * @return
//	 */
//	private static ComponentDescription getDescriptionForName(
//			ProcessConnection _connection, String _string) {
//
//		ComponentDescription desc = null;
//
//		desc = getDescriptionForName(_connection.m_senders, _string);
//
//		if (desc == null) {
//			desc = getDescriptionForName(_connection.m_receivers, _string);
//		}
//
//		return desc;
//
//	}

	/**
	 * @param _descriptions
	 * @param _string
	 * @return
	 */
	private static ComponentDescription getDescriptionForName(
			ComponentDescription[] _descriptions, String _string) {

		for (int i = 0; i < _descriptions.length; i++) {
			if (_descriptions[i].componentName.equals(_string)) {
				return _descriptions[i];
			}
		}
		return null;
	}


	
//	/**
//	 * Get descriptions from processes from previous configuration methods.
//	 * 
//	 * @param _processNames
//	 * @return
//	 * @throws ArchitectureConfigurationException
//	 */
//	private static ComponentDescription[] getComponentDescriptions(
//			ArrayList<String> _processNames)
//			throws ArchitectureConfigurationException {
//
//		// System.out.println("looking for " + _processNames.size() + "
//		// names");
//		// System.out.println("in " + m_subarchitectures.size() + "
//		// subarchs");
//
//		ArrayList<ProcessConnection> connections;
//		ComponentDescription[] descArray = new ComponentDescription[_processNames
//				.size()];
//		ComponentDescription desc = null;
//
//		// first get architecture
//		if (m_architecture != null) {
//
//			connections = m_architecture.getConnections();
//
//			// for each of the given names
//			for (int i = 0; i < descArray.length; i++) {
//
//				// look at each connection in turn
//				for (ProcessConnection connection : connections) {
//					desc = null;
//					desc = getDescriptionForName(connection, _processNames
//							.get(i));
//					if (desc != null) {
//						break;
//					}
//				}
//
//				// this still might be null at this point... check
//				// later!
//				descArray[i] = desc;
//			}
//		}
//
//		// System.out.println("in " + m_subarchitectures.size() + "
//		// subarchs");
//
//		// TODO optimise this process!
//
//		// now check subarchitectures in cases an architecture isn't
//		// used
//		for (int i = 0; i < descArray.length; i++) {
//
//			// System.out.println(i + ": " + descArray[i]);
//
//			// if it hasn't already been set
//			if (descArray[i] == null) {
//
//				// System.out.println(i + ": " + descArray[i]);
//
//				// for each subarchitecture
//				for (int j = 0; j < m_subarchitectures.size(); j++) {
//
//					SubarchitectureConfiguration subarch = m_subarchitectures
//							.get(j);
//
//					// System.out.println("subarch named: " +
//					// subarch.getName());
//
//					connections = subarch.getConnections();
//
//					// look at each connection in turn
//					for (ProcessConnection connection : connections) {
//
//						// System.out.println("named: " +
//						// _processNames.get(i));
//						descArray[i] = getDescriptionForName(connection,
//								_processNames.get(i));
//
//						if (descArray[i] != null) {
//							break;
//						}
//					}
//
//					if (descArray[i] != null) {
//						break;
//					}
//
//				}
//			}
//
//			// if still null here, check in extra descriptions
//			if (descArray[i] == null) {
//				for (ComponentDescription description : m_extraDescriptions) {
//					if (description.componentName.equals(_processNames.get(i))) {
//						descArray[i] = description;
//						break;
//					}
//				}
//			}
//
//			// if this is still null we're in trouble!
//			if (descArray[i] == null) {
//				throw new ArchitectureConfigurationException(
//						"Unknown process for extra connection: "
//								+ _processNames.get(i));
//			}
//
//			// System.err.println(descArray[i].componentName);
//			// System.err.println(descArray[i].m_hostName);
//			// System.err.println(descArray[i].m_className);
//
//		}
//
//		return descArray;
//	}

	/*
	 * Reset all stored values for next parse.
	 */
	private static void init() {
		m_subarchitectures = new ArrayList<SubarchitectureConfiguration>();
		m_componentNames = new HashSet<String>();
		m_subarchNames = new HashSet<String>();
		m_architecture = null;
		m_defaultHost = null;
		m_extraConnectionCount = 0;
		m_extraDescriptions = new ArrayList<ComponentDescription>();
		m_componentNumber = 0;
	}

	/**
	 * @param line
	 * @return
	 */
	private static boolean isHeader(String line) {
		return line.startsWith(SUBARCH_HEADER) || line.startsWith(ARCH_HEADER)
				|| line.startsWith(HOST_HEADER)
				|| line.startsWith(EXTRA_CONNECTION_HEADER)
				|| line.startsWith(EXTRA_COMPONENT_HEADER);
	}

	/**
	 * @param _lines
	 * @param _i
	 * @return
	 * @throws ArchitectureConfigurationException
	 */
	private static int parseArchitecture(ArrayList<String> _lines, int _i)
			throws ArchitectureConfigurationException {
		// System.out.println("start arch
		// parse-----------------------");

		String line;

		String archLine = _lines.get(_i);
		// System.out.println(archLine);

		m_architecture = parseArchitectureLine(archLine);

		int i = 0;
		for (i = _i + 1; i < _lines.size(); i++) {
			line = _lines.get(i);

			if (!(line.startsWith(COMMENT_CHAR) || line.length() == 0)) {
				// look for end of section
				if (isHeader(line)) {
					break;
				}
				else {
					parseArchitectureComponentLine(line, m_architecture);
				}
			}
		}

		// System.out.println("end arch parse-----------------------");
		return i - 1;
	}

	/**
	 * @param _line
	 * @param _m_architecture
	 * @throws ArchitectureConfigurationException
	 */
	private static void parseArchitectureComponentLine(String _line,
			ArchitectureConfiguration _m_architecture)
			throws ArchitectureConfigurationException {

//		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line, " ");
		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line);

		try {

			String lang = tokeniser.nextToken();
			String componentHost;

			// if not a flag then it's a host name
			if (!(lang.equals(CPP_FLAG) || lang.equals(JAVA_FLAG))) {
				componentHost = lang;
				lang = tokeniser.nextToken();
			}
			else {
				componentHost = m_defaultHost;
				if (m_defaultHost == null) {
					throw new ArchitectureConfigurationException(
							"No host specified for component. Use either HOST flag at top level or component host specification: "
									+ _line);
				}
				componentHost = m_defaultHost;
			}

			ComponentLanguage procLang;
			if (lang.equals(CPP_FLAG)) {
				procLang = ComponentLanguage.CPP;
			}
			else if (lang.equals(JAVA_FLAG)) {
				procLang = ComponentLanguage.JAVA;
			}
			else {
				throw new ArchitectureConfigurationException(
						"Unknown language flag: " + lang + "\nUse on of "
								+ CPP_FLAG + "|" + JAVA_FLAG + "\n" + _line);
			}

			// String componentType = tokeniser.nextToken();

			// if (componentType.equals(MG_FLAG)) {
			// parseMotiveGenerator(tokeniser, componentHost,
			// procLang, _m_architecture);
			// }
			// else if (componentType.equals(GM_FLAG)) {
			// parseGoalManager(tokeniser, componentHost, procLang,
			// _m_architecture);
			// }
			// else {
			throw new ArchitectureConfigurationException(
					"Unknown component type flag: " + lang + "\nUse on of "
							+ WM_FLAG + "|" + TM_FLAG + "|" + DD_FLAG + "|"
							+ UM_FLAG + "|" + GD_FLAG + "|" + MG_FLAG);
			// }

		}
		catch (NoSuchElementException e) {
			throw new ArchitectureConfigurationException(
					"Malformed subarch component line: " + _line, e);
		}

	}

	/**
	 * @param _archLine
	 * @return
	 */
	private static ArchitectureConfiguration parseArchitectureLine(
			String _archLine) {
		// don't actually parse anything yet
		return new ArchitectureConfiguration();
	}

	/**
	 * @param _tokeniser
	 * @return
	 * @throws ArchitectureConfigurationException
	 */
	private static Map<String, String> parseCommandLine(EnvVarTokenizer _tokeniser)
			throws ArchitectureConfigurationException {
		Map<String, String> prop = defaultConfig();

		String token;
		String key = null;
		String value = null;
		while (_tokeniser.hasMoreTokens()) {
			token = _tokeniser.nextToken();

			if (token.startsWith(COMMENT_CHAR)) {
				if (key != null) {
					prop.put(key, TRUE_VALUE);
				}
				break;
			}
			// if it's a command flag
			else if (token.startsWith("-")) {
				// if we have a previous key, assume its a single arg
				if (key != null) {
					prop.put(key, TRUE_VALUE);
				}

				// else store the key
				key = token;

			}
			else {
				// we have a value

				// if it's a string
				if (token.startsWith("\"") && (!token.endsWith("\""))) {
					// collect bits of string into one
					value = token;
					while (_tokeniser.hasMoreTokens()) {
						token = _tokeniser.nextToken();

						// last bit of string
						if (token.endsWith("\"")) {
							value += " " + token;

							// strip off leading and trailing "s
							value = value.substring(1, value.length() - 1);

							prop.put(key, value);
							key = null;
							break;
						}
						// contents of string
						else {
							value += " " + token;
						}
					}

				}
				else {
					if (key != null) {
						// if the token looks like "fsdfd"
						if (token.startsWith("\"") && token.endsWith("\"")) {
							token = token.substring(1, token.length() - 1);
						}

						prop.put(key, token);
						key = null;
					}
					else {
						throw new ArchitectureConfigurationException(
								"Malformed command line args. Stray value: "
										+ token);
					}
				}

			}

		}

		// if we have a key here, it has to be no-valued
		if (key != null) {
			prop.put(key, TRUE_VALUE);
		}

		// let's add the config file as well ;)

		prop.put(COMPONENTNUMBERKEY.value, "" + m_componentNumber++);

		// prop.list(System.out);

		return prop;
	}

	private static int m_componentNumber;

	public static ArrayList<ComponentDescription> m_extras;

	/**
	 * Add some default properties into the component
	 * 
	 * @return
	 */
	private static Map<String, String> defaultConfig() {
		Map<String, String> prop = new HashMap<String, String>();
		// assert (m_currentFile != null);
		if (m_currentFile != null) {
			prop.put(CONFIGFILEKEY.value, m_currentFile);
		}
		return prop;
	}

	/**
	 * @param _tokeniser
	 * @param _componentHost
	 * @param _procLang
	 * @param _subarch
	 */
	private static void parseUnmanagedComponent(EnvVarTokenizer _tokeniser,
			String _componentHost, ComponentLanguage _procLang,
			SubarchitectureConfiguration _subarch)
			throws ArchitectureConfigurationException {

		String componentName = _tokeniser.nextToken();
		checkComponentName(componentName);

		String className = _tokeniser.nextToken();

		Map<String, String> config = parseCommandLine(_tokeniser);

		_subarch.addUnmanagedComponent(componentName, className,
				_procLang, _componentHost, config);

	}

	private static void checkComponentName(String _componentName)
			throws ArchitectureConfigurationException {
		if (m_componentNames.contains(_componentName)) {
			throw new ArchitectureConfigurationException(
					"Duplicate component names not allowed: " + _componentName);
		}
		else {
			m_componentNames.add(_componentName);
		}
	}

	private static void checkSubarchitectureName(String _subarchName)
			throws ArchitectureConfigurationException {
		if (m_subarchNames.contains(_subarchName)) {
			throw new ArchitectureConfigurationException(
					"Duplicate subarchitecture names not allowed: "
							+ _subarchName);
		}
		else {
			m_subarchNames.add(_subarchName);
		}
	}

	/**
	 * @param _line
	 * @return
	 * @throws ArchitectureConfigurationException
	 */
	private static ComponentDescription parseExtraComponent(String _line)
			throws ArchitectureConfigurationException {
		try {

			System.out.println("CASTConfigParser.parseExtraComponent(): "
					+ _line);

//			EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line, " ");
			EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line);

			// skip header
			tokeniser.nextToken();

			String lang = tokeniser.nextToken();
			String componentHost;

			// if not a flag then it's a host name
			if (!(lang.equals(CPP_FLAG) || lang.equals(JAVA_FLAG))) {
				componentHost = lang;
				lang = tokeniser.nextToken();
			}
			else {
				if (m_defaultHost == null) {
					throw new ArchitectureConfigurationException(
							"No host specified for component. Use either HOST flag at top level or component host specification: "
									+ _line);
				}
				componentHost = m_defaultHost;
			}

			System.err.println("componentHost: " + componentHost);

			ComponentLanguage procLang;
			if (lang.equals(CPP_FLAG)) {
				procLang = ComponentLanguage.CPP;
			}
			else if (lang.equals(JAVA_FLAG)) {
				procLang = ComponentLanguage.JAVA;
			}
			else {
				throw new ArchitectureConfigurationException(
						"Unknown language flag: " + lang + "\nUse on of "
								+ CPP_FLAG + "|" + JAVA_FLAG + "\n" + _line);
			}

			String componentName = tokeniser.nextToken();
			checkComponentName(componentName);

			String className = tokeniser.nextToken();

			Map<String, String> config = parseCommandLine(tokeniser);

			// also add config file to config!!
			// config.put(CAATConfigParser.CONFIG_FILE_FLAG,
			// m_lastParse);

			return new ComponentDescription(componentName, className,
					procLang, componentHost, config);

		}
		catch (NoSuchElementException e) {
			throw new ArchitectureConfigurationException(
					"Malformed subarch component line: \"" + _line + "\"", e);
		}
	}

//	/**
//	 * @param _line
//	 * @return
//	 * @throws ArchitectureConfigurationException
//	 */
//	private static ProcessConnection parseExtraConnection(String _line)
//			throws ArchitectureConfigurationException {
//
//		// System.out.println("CAATConfigParser.parseExtraConnection():
//		// "
//		// + _line);
//
//		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line, " ");
//
//		// get rid of extra flag
//		tokeniser.nextToken();
//
//		// get connection datatype
//		String datatype = tokeniser.nextToken();
//		String connectionID = "";
//
//		ArrayList<String> senders = new ArrayList<String>();
//		ArrayList<String> receivers = new ArrayList<String>();
//
//		// process descriptions senders
//		String token;
//		FrameworkConnectionType connectionType = null;
//		while (tokeniser.hasMoreTokens()) {
//			token = tokeniser.nextToken();
//			if (token.equals(PUSH_CONNECTION_FLAG)) {
//				connectionType = FrameworkConnectionType.PUSH_CONNECTION;
//				connectionID = EXTRA_CONNECTION_ID + ":"
//						+ (m_extraConnectionCount++) + ":push";
//
//			}
//			else if (token.equals(PULL_CONNECTION_FLAG)) {
//				connectionType = FrameworkConnectionType.PULL_CONNECTION;
//				connectionID = EXTRA_CONNECTION_ID + ":"
//						+ (m_extraConnectionCount++) + ":pull";
//			}
//			// if no connection type yet, add to senders
//			else if (connectionType == null) {
//				senders.add(token);
//			}
//			else {
//				receivers.add(token);
//			}
//		}
//
//		if (connectionType == null) {
//			throw new ArchitectureConfigurationException(
//					"No connection type specified for extra connection: \""
//							+ _line + "\"");
//		}
//		else if (senders.isEmpty()) {
//			throw new ArchitectureConfigurationException(
//					"No sender processes specified for extra connection: \""
//							+ _line + "\"");
//		}
//		else if (receivers.isEmpty()) {
//			throw new ArchitectureConfigurationException(
//					"No receiver processes specified for extra connection: \""
//							+ _line + "\"");
//		}
//
//		ComponentDescription[] senderDescriptions = getComponentDescriptions(senders);
//		ComponentDescription[] receiverDescriptions = getComponentDescriptions(receivers);
//
//		return new ProcessConnection(senderDescriptions, receiverDescriptions,
//				connectionType, datatype, connectionID);
//	}

	/**
	 * @param _lines
	 * @throws ArchitectureConfigurationException
	 */
	private static ArrayList<ComponentDescription> parseExtras(
			ArrayList<String> _lines) throws ArchitectureConfigurationException {

		ArrayList<ComponentDescription> connections = new ArrayList<ComponentDescription>();

		for (int i = 0; i < _lines.size(); i++) {

			String line = (String) _lines.get(i);

			// System.out.println("line: " + line);

			if (!line.startsWith(COMMENT_CHAR)) {

//				if (line.startsWith(EXTRA_CONNECTION_HEADER)) {
//					connections.add(parseExtraConnection(line));
//				}
//				else
				if (line.startsWith(EXTRA_COMPONENT_HEADER)) {
					connections.add(parseExtraComponent(line));
				}
			}
		}

		return connections;

	}

	/**
	 * @param _tokeniser
	 * @param _componentHost
	 * @param _procLang
	 * @param _subarch
	 */
	private static void parseManagedComponent(EnvVarTokenizer _tokeniser,
			String _componentHost, ComponentLanguage _procLang,
			SubarchitectureConfiguration _subarch)
			throws ArchitectureConfigurationException {

		String componentName = _tokeniser.nextToken();
		checkComponentName(componentName);

		String className = _tokeniser.nextToken();

		Map<String, String> config = parseCommandLine(_tokeniser);

		_subarch.addManagedComponent(componentName, className,
				_procLang, _componentHost, config);

	}

	// /**
	// * @param _tokeniser
	// * @param _componentHost
	// * @param _procLang
	// * @param _m_architecture
	// * @throws ArchitectureConfigurationException
	// */
	// private static void parseGoalManager(EnvVarTokenizer _tokeniser,
	// String _componentHost, ComponentLanguage _procLang,
	// ArchitectureConfiguration _m_architecture)
	// throws ArchitectureConfigurationException {
	//
	// String className = _tokeniser.nextToken();
	// Map<String, String> config = parseCommandLine(_tokeniser);
	//
	// _m_architecture.setGoalManager(className, _procLang,
	// _componentHost, config);
	//
	// }

	/**
	 * @param _line
	 * @throws ArchitectureConfigurationException
	 */
	private static void parseHost(String _line)
			throws ArchitectureConfigurationException {
//		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line, " ");
		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line);

		// get rid of HOST
		tokeniser.nextToken();

		try {
			String hostname = tokeniser.nextToken();

			if (hostname.startsWith(COMMENT_CHAR)) {
				throw new ArchitectureConfigurationException(
						"Malformed host line: " + _line);
			}
			else {
				// System.out.println("setting default hostname: " + hostname);
				m_defaultHost = hostname;
			}
		}
		catch (NoSuchElementException e) {
			throw new ArchitectureConfigurationException(
					"Malformed host line: " + _line, e);
		}

	}

	/**
	 * @param _lines
	 * @throws ArchitectureConfigurationException
	 */
	private static void parseLines(ArrayList<String> _lines)
			throws ArchitectureConfigurationException {

		for (int i = 0; i < _lines.size(); i++) {

			String line = (String) _lines.get(i);

//			 System.out.println("line: " + line);

			if (!line.startsWith(COMMENT_CHAR)) {

				if (line.startsWith(SUBARCH_HEADER)) {
					i = parseSubarchitecture(_lines, i);
				}
				else if (line.startsWith(ARCH_HEADER)) {
					i = parseArchitecture(_lines, i);
				}
				else if (line.startsWith(HOST_HEADER)) {
					parseHost(line);
				}
			}
		}

	}

	// /**
	// * @param _tokeniser
	// * @param _componentHost
	// * @param _procLang
	// * @param _m_architecture
	// */
	// private static void parseMotiveGenerator(
	// EnvVarTokenizer _tokeniser, String _componentHost,
	// ComponentLanguage _procLang,
	// ArchitectureConfiguration _m_architecture)
	// throws ArchitectureConfigurationException {
	//
	// String className = _tokeniser.nextToken();
	// Map<String, String> config = parseCommandLine(_tokeniser);
	//
	// _m_architecture.setMotiveGenerator(className, _procLang,
	// _componentHost, config);
	//
	// }

	/**
	 * @param _lines
	 * @param _i
	 * @return
	 * @throws ArchitectureConfigurationException
	 */
	private static int parseSubarchitecture(ArrayList<String> _lines, int _i)
			throws ArchitectureConfigurationException {

//		 System.out
//		 .println("start subarch parse-----------------------");
		String line;

		String subarchLine = _lines.get(_i);
		// System.out.println(subarchLine);
		SubarchitectureConfiguration subarch = parseSubarchitectureLine(subarchLine);

		int i = _i + 1;
		for (i = _i + 1; i < _lines.size(); i++) {
			line = _lines.get(i);

			if (!(line.startsWith(COMMENT_CHAR) || line.length() == 0)) {
				// look for end of section
				if (isHeader(line)) {
					break;
				}
				else {
					// System.out.println(line);
					parseSubarchitectureComponentLine(line, subarch);
				}
			}
		}
		// System.out.println("end subarch
		// parse-----------------------");

		m_subarchitectures.add(subarch);
		// System.out.println("subarch count: "
		// + m_subarchitectures.size());

		return i - 1;
	}

	/**
	 * @param _line
	 * @param _subarch
	 * @throws ArchitectureConfigurationException
	 */
	private static void parseSubarchitectureComponentLine(String _line,
			SubarchitectureConfiguration _subarch)
			throws ArchitectureConfigurationException {

//		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line, " \t");
		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_line);

		try {

			String lang = tokeniser.nextToken();
			String componentHost;

			// if not a flag then it's a host name
			if (!(lang.equals(CPP_FLAG) || lang.equals(JAVA_FLAG))) {
				componentHost = lang;
				lang = tokeniser.nextToken();
			}
			else {
				componentHost = _subarch.getHost();
				if (componentHost == null) {
					throw new ArchitectureConfigurationException(
							"No host specified for component. Use either HOST flag at top level, subarchitecture host specification or component host specification: "
									+ _line);
				}
			}

			ComponentLanguage procLang;
			if (lang.equals(CPP_FLAG)) {
				procLang = ComponentLanguage.CPP;
			}
			else if (lang.equals(JAVA_FLAG)) {
				procLang = ComponentLanguage.JAVA;
			}
			else {
				throw new ArchitectureConfigurationException(
						"Unknown language flag: " + lang + "\nUse on of "
								+ CPP_FLAG + "|" + JAVA_FLAG + "\n" + _line);
			}

			String componentType = tokeniser.nextToken();

			if (componentType.equals(WM_FLAG)) {
				parseWorkingMemory(tokeniser, componentHost, procLang, _subarch);
			}
			else if (componentType.equals(TM_FLAG)) {
				parseTaskManager(tokeniser, componentHost, procLang, _subarch);
			}
			else if (componentType.equals(DD_FLAG)
					|| componentType.equals(UM_FLAG)) {
				parseUnmanagedComponent(tokeniser, componentHost, procLang,
						_subarch);
			}
			else if (componentType.equals(GD_FLAG)
					|| componentType.equals(MG_FLAG)) {
				parseManagedComponent(tokeniser, componentHost, procLang,
						_subarch);
			}
			else {
				throw new ArchitectureConfigurationException(
						"Unknown component type flag: " + lang + "\nUse on of "
								+ WM_FLAG + "|" + TM_FLAG + "|" + DD_FLAG + "|"
								+ UM_FLAG + "|" + GD_FLAG + "|" + MG_FLAG);
			}

		}
		catch (NoSuchElementException e) {
			throw new ArchitectureConfigurationException(
					"Malformed subarch component line: \"" + _line + "\"", e);
		}
	}

	/**
	 * @param _subarchLine
	 * @return
	 * @throws ArchitectureConfigurationException
	 */
	private static SubarchitectureConfiguration parseSubarchitectureLine(
			String _subarchLine) throws ArchitectureConfigurationException {

//		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_subarchLine, " ");
		EnvVarTokenizer tokeniser = new EnvVarTokenizer(_subarchLine);

		// get rid of SUBARCHITECTURE
		tokeniser.nextToken();

		try {
			String subarchName = tokeniser.nextToken();
			checkSubarchitectureName(subarchName);

			if (subarchName.startsWith(COMMENT_CHAR)) {
				throw new ArchitectureConfigurationException(
						"Malformed subarch line: " + _subarchLine);
			}

			String subarchHost;
			if (tokeniser.hasMoreTokens()) {
				subarchHost = tokeniser.nextToken();
				if (subarchHost.startsWith(COMMENT_CHAR)) {
					subarchHost = m_defaultHost;
				}
			}
			else {
				subarchHost = m_defaultHost;
			}

			return new SubarchitectureConfiguration(subarchName, subarchHost);

		}
		catch (NoSuchElementException e) {
			throw new ArchitectureConfigurationException(
					"Malformed subarch line: " + _subarchLine, e);
		}
	}

	/**
	 * @param _tokeniser
	 * @param _componentHost
	 * @param _procLang
	 * @param _subarch
	 * @throws ArchitectureConfigurationException
	 */
	private static void parseTaskManager(EnvVarTokenizer _tokeniser,
			String _componentHost, ComponentLanguage _procLang,
			SubarchitectureConfiguration _subarch)
			throws ArchitectureConfigurationException {

		String className = _tokeniser.nextToken();
		Map<String, String> config = parseCommandLine(_tokeniser);

		_subarch.setTaskManager(className, _procLang, _componentHost, config);
	}

	/**
	 * @param _tokeniser
	 * @param _componentHost
	 * @param _procLang
	 * @param __subarch
	 * @throws ArchitectureConfigurationException
	 */
	private static void parseWorkingMemory(EnvVarTokenizer _tokeniser,
			String _componentHost, ComponentLanguage _procLang,
			SubarchitectureConfiguration __subarch)
			throws ArchitectureConfigurationException {
		String className = _tokeniser.nextToken();
		Map<String, String> config = parseCommandLine(_tokeniser);

		__subarch
				.setWorkingMemory(className, _procLang, _componentHost, config);
	}

	/**
	 * @param _filename
	 * @return
	 * @throws FileNotFoundException
	 * @throws IOException
	 */
	private static ArrayList<String> readFile(String _filename)
			throws FileNotFoundException, IOException {

		File configFile = new File(_filename);
		FileReader fileRead = new FileReader(configFile);
		BufferedReader reader = new BufferedReader(fileRead);
		ArrayList<String> lines = new ArrayList<String>();
		StringBuffer sb = new StringBuffer();

		String line;
		while (reader.ready()) {
			line = reader.readLine().trim();
			lines.add(line);
			sb.append(line);
			sb.append("\n");
		}

		m_lastParse = sb.toString();

		return lines;
	}

	/**
	 * @param _filename
	 * @return
	 * @throws FileNotFoundException
	 * @throws IOException
	 */
	private static ArrayList<String> readString(String _config)
			throws IOException {

		StringReader stringRead = new StringReader(_config);
		BufferedReader reader = new BufferedReader(stringRead);
		ArrayList<String> lines = new ArrayList<String>();
		StringBuffer sb = new StringBuffer();

		String line = reader.readLine();
		while (line != null) {
			line = line.trim();
			// System.out.println(line);

			lines.add(line);
			sb.append(line);
			sb.append("\n");

			line = reader.readLine();
		}

		m_lastParse = sb.toString();

		return lines;
	}

//	public static void main(String[] args)
//			throws ArchitectureConfigurationException {
//		CASTConnectionConfiguration ccc = parseConfigFile(args[0]);
//
//		ArrayList<ProcessConnection> procs = ccc.getConnections();
//
//		// procs.addAll(parseExtras(args[0]));
//
//		for (ProcessConnection connection : procs) {
//			System.out.println(connection.m_connectionID);
//		}
//
//	}

	public static void parseConfigFile(String _filename)
			throws ArchitectureConfigurationException {

		try {
			init();
			ArrayList<String> lines = readFile(_filename);
			m_currentFile = _filename;
			// setup container to hold everything
			m_architecture = new ArchitectureConfiguration();
			
			 processLines(lines);
			 m_extras = parseExtras(lines);
		}
		catch (FileNotFoundException e) {
			throw new ArchitectureConfigurationException(
					"Config file does not exist: " + _filename, e);
		}
		catch (IOException e) {
			throw new ArchitectureConfigurationException(
					"Error parsing config file: " + _filename, e);
		}

	}

	public static ArchitectureConfiguration parseConfigString(String _config)
			throws ArchitectureConfigurationException {
		try {
			// System.out.println(_config);
			init();
			m_architecture = new ArchitectureConfiguration();
			ArrayList<String> lines = readString(_config);
			return processLines(lines);
		}
		catch (IOException e) {
			throw new ArchitectureConfigurationException(
					"Error parsing config file: " + _config, e);
		}
	}

	/**
	 * @param lines
	 * @return
	 * @throws ArchitectureConfigurationException
	 */
	private static ArchitectureConfiguration processLines(
			ArrayList<String> lines) throws ArchitectureConfigurationException {
		parseLines(lines);

		if (m_architecture == null) {

			if (m_subarchitectures.isEmpty()) {
				throw new ArchitectureConfigurationException(
						"No architecture or subarchitecture configuration information found");
			}

			SubarchitectureConfiguration singleSubarch = m_subarchitectures
					.get(0);
			System.err
					.println("WARNING: No full architecture information found, configuring with first subarchitecture found = "
							+ singleSubarch.getName());

			ArchitectureConfiguration singleSAArch = new ArchitectureConfiguration();
			singleSAArch.addSubarchitectureConfiguration(singleSubarch);

			return singleSAArch;

		}
		else {

			//nah: allow empty
//			if (m_subarchitectures.isEmpty()) {
//				throw new ArchitectureConfigurationException(
//						"No subarchitecture configuration information found");
//			}

			for (SubarchitectureConfiguration subarch : m_subarchitectures) {
				m_architecture.addSubarchitectureConfiguration(subarch);
			}

			additionalConfiguration(m_architecture);

			return m_architecture;
		}
	}

	/**
	 * Add any config data that can only be sorted after the parse
	 * 
	 * @param _m_architecture
	 */
	private static void additionalConfiguration(
			ArchitectureConfiguration _architecture) {

		Map<String, String> props = new HashMap<String, String>();
		addIDs(_architecture, props);

		ArrayList<SubarchitectureConfiguration> subarchs = _architecture
				.getSubarchitectures();
		for (SubarchitectureConfiguration subarch : subarchs) {
			subarch.addConfigurationInformation(props);
		}

	}

	private static void addIDs(ArchitectureConfiguration _architecture,
			Map<String, String> _props) {
		ArrayList<SubarchitectureConfiguration> subarchs = _architecture
				.getSubarchitectures();
		String wmIDs = "";
		String componentIDs = "";

		for (SubarchitectureConfiguration subarch : subarchs) {
			wmIDs += "," + subarch.getWorkingMemoryConfig().componentName;
			ArrayList<ComponentDescription> umcs = subarch
					.getUnmanagedComponents();
			for (ComponentDescription desc : umcs) {
				componentIDs += "," + desc.componentName;
			}
			ArrayList<ComponentDescription> mcs = subarch.getManagedComponents();
			for (ComponentDescription desc : mcs) {
				componentIDs += "," + desc.componentName;
			}
			componentIDs += "," + subarch.getTaskManagerConfig().componentName;
		}

		_props.put(WMIDSKEY.value, wmIDs);
		_props.put(COMPONENTIDSKEY.value, componentIDs);

	}

//	public static ProcessConnection[] parseExtras(String _filename)
//			throws ArchitectureConfigurationException {
//
//		try {
//			ArrayList<String> lines = readFile(_filename);
//
//			ArrayList<ProcessConnection> extras = parseExtras(lines);
//
//			return (ProcessConnection[]) extras
//					.toArray(new ProcessConnection[extras.size()]);
//		}
//		catch (FileNotFoundException e) {
//			throw new ArchitectureConfigurationException(
//					"Config file does not exist: " + _filename, e);
//		}
//		catch (IOException e) {
//			throw new ArchitectureConfigurationException(
//					"Error parsing config file: " + _filename, e);
//		}
//
//	}

	/**
	 * @return
	 */
	public static String getLastParse() {
		return m_lastParse;
	}

}
