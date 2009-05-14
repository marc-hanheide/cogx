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

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Properties;

import balt.corba.autogen.FrameworkBasics.FrameworkConnectionType;
import balt.corba.autogen.FrameworkBasics.ProcessConnection;
import balt.corba.autogen.FrameworkBasics.ProcessDescription;
import cast.cdl.GLOBAL_CONNECTION_PREFIX;
import cast.cdl.SUBARCH_ID_KEY;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.XARCH_CONNECTION_PREFIX;
import cast.core.CASTUtils;

/**
 * @author nah
 */
public class ArchitectureConfiguration implements CASTConnectionConfiguration,
		Iterable<SubarchitectureConfiguration> {

	/**
	 * 
	 */
	public static final String MOTIVE_GEN_NAM = "motivegen";

	/**
	 * 
	 */
	public static final String GOAL_MANAGER_NAME = "goalmanager";

	public static final String XARCH_PREFIX = XARCH_CONNECTION_PREFIX.value;

	public static final String GLOBAL_PREFIX = GLOBAL_CONNECTION_PREFIX.value;

	private ArrayList<SubarchitectureConfiguration> m_subarchitectures;

	// private ProcessDescription m_motiveGenerator;

	// private ProcessDescription m_goalManager;
	private Properties m_archDefaults;

	// public boolean isSingleSubarchConfig() {
	// return m_motiveGenerator == null;
	// }
	//
	// /**
	// * @return the motiveGenerator
	// */
	// public ProcessDescription getMotiveGenerator() {
	// return m_motiveGenerator;
	// }
	//
	// /**
	// * @return the goalManager
	// */
	// public ProcessDescription getGoalManager() {
	// return m_goalManager;
	// }

	/**
	 * @param _string
	 */
	public ArchitectureConfiguration() {
		m_subarchitectures = new ArrayList<SubarchitectureConfiguration>();
		m_archDefaults = new Properties();
		m_archDefaults.setProperty(SUBARCH_ID_KEY.value, "general.arch");

	}

	public ArrayList<SubarchitectureConfiguration> getSubarchitectures() {
		return m_subarchitectures;
	}
	
	//
	// /**
	// * @param _host
	// * @param _string
	// * @param _string2
	// */
	// public ArchitectureConfiguration(String _host, String _motiveGen,
	// String _goalMan) {
	// this();
	// setMotiveGenerator(_motiveGen, ProcessLanguage.JAVA_PROCESS,
	// _host);
	// setGoalManager(_goalMan, ProcessLanguage.JAVA_PROCESS, _host);
	// }

	// public void setMotiveGenerator(String _motiveGen,
	// ProcessLanguage _lang, String _host) {
	// setMotiveGenerator(_motiveGen, _lang, _host, new Properties());
	// }
	//
	// public void setMotiveGenerator(String _motiveGen,
	// ProcessLanguage _lang, String _host, Properties _props) {
	// Properties props = new Properties(m_archDefaults);
	// props.putAll(_props);
	//
	// m_motiveGenerator = ClientUtils.newProcessDescription(
	// // unique id for the process
	// ArchitectureConfiguration.MOTIVE_GEN_NAM,
	// // actual class name of the process
	// _motiveGen,
	// // indicate that the process is in Java
	// _lang,
	// // and what machine it should run on
	// _host, props);
	//
	// }
	//
	// public void setGoalManager(String _goalManager,
	// ProcessLanguage _lang, String _host) {
	// setGoalManager(_goalManager, _lang, _host, new Properties());
	// }
	//
	// public void setGoalManager(String _goalManager,
	// ProcessLanguage _lang, String _host, Properties _props) {
	//
	// Properties props = new Properties(m_archDefaults);
	// props.putAll(_props);
	//
	// m_goalManager = ClientUtils.newProcessDescription(
	// // unique id for the process
	// ArchitectureConfiguration.GOAL_MANAGER_NAME,
	// // actual class name of the process
	// _goalManager,
	// // indicate that the process is in Java
	// _lang,
	// // and what machine it should run on
	// _host, props);
	//
	// }

	public void addSubarchitectureConfiguration(
			SubarchitectureConfiguration _config) {
		m_subarchitectures.add(_config);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see CAST.configuration.CASTConnectionConfiguration#getConnections()
	 */
	public ArrayList<ProcessConnection> getConnections()
			throws ArchitectureConfigurationException {

		// if (m_goalManager == null) {
		// throw new ArchitectureConfigurationException(
		// "Goal Manager configuration not set.");
		// }
		// if (m_motiveGenerator == null) {
		// throw new ArchitectureConfigurationException(
		// "Motive Generator configuration not set.");
		// }

		// if (m_goalManager == null || m_motiveGenerator == null) {
		// System.err.println("Warning: Just using single
		// subarchitecture");
		// return m_subarchitectures.get(0).getConnections();
		// }

		ArrayList<ProcessConnection> connections = new ArrayList<ProcessConnection>();

		// collect goal managers
		ArrayList<ProcessDescription> taskManagers = new ArrayList<ProcessDescription>();
		// collect working memories
		ArrayList<ProcessDescription> workingMemories = new ArrayList<ProcessDescription>();

		// for each sub arch
		for (SubarchitectureConfiguration subarch : m_subarchitectures) {

			// add connections from sub-archs
			try {
				connections.addAll(subarch.getConnections());
			} catch (ArchitectureConfigurationException e) {
				throw new ArchitectureConfigurationException(
						"subarchitecture configuration error: " + subarch, e);
			}

			// collect other connections
			taskManagers.add(subarch.getTaskManagerConfig());
			workingMemories.add(subarch.getWorkingMemoryConfig());
		}

		if (m_subarchitectures.size() > 1) {

			ProcessDescription[] wms = workingMemories
					.toArray(new ProcessDescription[workingMemories.size()]);
			
			//TODO: refactor to use n connections... this will be a bottleneck!!!
			// to save on overhead, change filter sending shares a single connection
			connections.add(new ProcessConnection(wms, wms,
					FrameworkConnectionType.PUSH_CONNECTION,
					CASTUtils.typeName(WorkingMemoryChangeFilter.class), XARCH_PREFIX + ":wmcf"));

			ArrayList<ProcessDescription> workingMemoriesCopy = new ArrayList<ProcessDescription>();

			workingMemoriesCopy.addAll(workingMemories);

			int workingMemoriesExceptOne = workingMemoriesCopy.size() - 1;

			for (ProcessDescription wm : workingMemories) {
				// remove from list
				workingMemoriesCopy.remove(wm);

				ProcessDescription[] processes = new ProcessDescription[workingMemoriesExceptOne];
				workingMemoriesCopy.toArray(processes);

				// TODO Optimise using local data connections where appropriate

				connections.add(new ProcessConnection(processes,
						new ProcessDescription[] { wm },
						FrameworkConnectionType.PULL_CONNECTION,
						CASTUtils.typeName(WorkingMemoryEntry[].class), XARCH_PREFIX + ":"
								+ wm.m_processName + ":wmpull"));

				connections.add(new ProcessConnection(processes,
						new ProcessDescription[] { wm },
						FrameworkConnectionType.PUSH_CONNECTION,
						CASTUtils.typeName(WorkingMemoryEntry.class), XARCH_PREFIX + ":"
								+ wm.m_processName + ":wmpush"));

				connections.add(new ProcessConnection(processes,
						new ProcessDescription[] { wm },
						FrameworkConnectionType.PUSH_CONNECTION,
						CASTUtils.typeName(WorkingMemoryChange.class), XARCH_PREFIX + ":"
								+ wm.m_processName + ":wmchanges"));

				//begin additions for locking
				connections.add(new ProcessConnection(processes,
						new ProcessDescription[] { wm },
						FrameworkConnectionType.PULL_CONNECTION,
						CASTUtils.typeName(WorkingMemoryPermissions.class), XARCH_PREFIX + ":"
								+ wm.m_processName + ":wmpermissions"));
				//end additions for locking				
				
				workingMemoriesCopy.add(wm);
			}
		}

		// add global change connection
		ProcessDescription[] workingMemoriesArray = new ProcessDescription[workingMemories
				.size()];
		workingMemories.toArray(workingMemoriesArray);

		return connections;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Iterable#iterator()
	 */
	public Iterator<SubarchitectureConfiguration> iterator() {
		return m_subarchitectures.iterator();
	}

}
