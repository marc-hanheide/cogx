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
package cast.configuration;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Properties;

import balt.clients.ClientUtils;
import balt.corba.autogen.FrameworkBasics.FrameworkConnectionType;
import balt.corba.autogen.FrameworkBasics.ProcessConfigurationMap;
import balt.corba.autogen.FrameworkBasics.ProcessConnection;
import balt.corba.autogen.FrameworkBasics.ProcessDescription;
import balt.corba.autogen.FrameworkBasics.ProcessLanguage;
import cast.architecture.subarchitecture.SubarchitectureTaskManager;
import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;
import cast.cdl.InformationProcessingTask;
import cast.cdl.SUBARCH_ID_KEY;
import cast.cdl.TaskDescription;
import cast.cdl.TaskManagementResult;
import cast.cdl.TaskResult;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import cast.core.data.CASTWorkingMemoryEntry;

/**
 * @author nah
 */
public class SubarchitectureConfiguration implements
		CASTConnectionConfiguration {

	private ArrayList<ProcessDescription> m_unmanagedProcesses;

	private ArrayList<ProcessDescription> m_managedProcesses;

	private ProcessDescription m_taskManager;

	private ProcessDescription m_workingMemory;

	private String m_said;

	private Properties m_subarchDefaults;

	private String m_host;


	/**
	 * @return the taskManager
	 */
	public ProcessDescription getTaskManagerConfig() {
		return m_taskManager;
	}

	/**
	 * @return the said
	 */
	public String getID() {
		return m_said;
	}

	/**
	 * 
	 */
	public SubarchitectureConfiguration(String _said) {
		this(_said, "no host");
	}

	/**
	 * 
	 */
	public SubarchitectureConfiguration(String _said, String _host) {
		m_said = _said;
		m_host = _host;
		m_managedProcesses = new ArrayList<ProcessDescription>(1);
		m_unmanagedProcesses = new ArrayList<ProcessDescription>(1);

		m_subarchDefaults = new Properties();
		m_subarchDefaults.setProperty(SUBARCH_ID_KEY.value, _said);
	}

	/**
	 * @param _workingMemoryClass
	 * @param _lang
	 */
	public void setWorkingMemory(String _workingMemoryClass,
			ProcessLanguage _lang, String _host) {
		setWorkingMemory(_workingMemoryClass, _lang, _host, new Properties());
	}

	/**
	 * @param _workingMemoryClass
	 * @param _lang
	 */
	public void setWorkingMemory(String _workingMemoryClass,
			ProcessLanguage _lang, String _host, Properties _props) {

		Properties props = new Properties(m_subarchDefaults);
		props.putAll(_props);

		m_workingMemory = ClientUtils.newProcessDescription(
		// unique id for the process
				SubarchitectureWorkingMemory.createID(m_said),
				// actual class name of the process
				_workingMemoryClass,
				// indicate that the process is in Java
				_lang,
				// and what machine it should run on
				_host, props);

		// System.out.println("props: " +
		// props.getProperty(SUBARCH_ID_KEY.value));
	}

	/**
	 * @param _taskManagerClass
	 * @param _lang
	 */
	public void setTaskManager(String _taskManagerClass, ProcessLanguage _lang,
			String _host) {
		setTaskManager(_taskManagerClass, _lang, _host, new Properties());
	}

	public void setTaskManager(String _taskManagerClass, ProcessLanguage _lang,
			String _host, Properties _props) {

		Properties props = new Properties(m_subarchDefaults);
		props.putAll(_props);

		m_taskManager = ClientUtils.newProcessDescription(
		// unique id for the process
				SubarchitectureTaskManager.createID(m_said),
				// actual class name of the process
				_taskManagerClass,
				// indicate that the process is in Java
				_lang,
				// and what machine it should run on
				_host, props);

	}

	/**
	 * @return the dataDrivenProcess
	 */
	public ArrayList<ProcessDescription> getUnmanagedComponents() {
		return m_unmanagedProcesses;
	}

	/**
	 * @return the goalDrivenProcess
	 */
	public ArrayList<ProcessDescription> getManagedComponents() {
		return m_managedProcesses;
	}

	public void addUnmanagedProcessingComponent(String _name, String _class,
			ProcessLanguage _lang, String _host) {
		addUnmanagedProcessingComponent(_name, _class, _lang, _host,
				new Properties());
	}


	public void addUnmanagedProcessingComponent(String _name, String _class,
			ProcessLanguage _lang, String _host, Properties _props) {

		Properties props = new Properties(m_subarchDefaults);
		props.putAll(_props);

		m_unmanagedProcesses.add(ClientUtils.newProcessDescription(_name,
				_class, _lang, _host, props));

	}

	public void addManagedProcessingComponent(String _name, String _class,
			ProcessLanguage _lang, String _host) {
		addManagedProcessingComponent(_name, _class, _lang, _host,
				new Properties());
	}

	public void addManagedProcessingComponent(String _name, String _class,
			ProcessLanguage _lang, String _host, Properties _props) {

		Properties props = new Properties(m_subarchDefaults);
		props.putAll(_props);

		m_managedProcesses.add(ClientUtils.newProcessDescription(_name, _class,
				_lang, _host, props));

	}

	public ProcessDescription getWorkingMemoryConfig() {
		return m_workingMemory;
	}

	public ArrayList<ProcessConnection> getConnections()
			throws ArchitectureConfigurationException {

		if (m_workingMemory == null) {
			throw new ArchitectureConfigurationException(
					"Working Memory config is not set");
		}

		if (m_taskManager == null) {
			throw new ArchitectureConfigurationException(
					"Task Manager config is not set");
		}

		ArrayList<ProcessConnection> connections = new ArrayList<ProcessConnection>();

		ArrayList<ProcessDescription> allProcessesArrayList = new ArrayList<ProcessDescription>();
		allProcessesArrayList.addAll(m_unmanagedProcesses);
		allProcessesArrayList.addAll(m_managedProcesses);

		ArrayList<ProcessDescription> allWMLocalProcessesArraylist = new ArrayList<ProcessDescription>();
		ArrayList<ProcessDescription> allWMRemoteProcessesArraylist = new ArrayList<ProcessDescription>();

		for (ProcessDescription description : allProcessesArrayList) {
			if (description.m_language == m_workingMemory.m_language
					&& description.m_hostName
							.equals(m_workingMemory.m_hostName)) {
				allWMLocalProcessesArraylist.add(description);
			} else {
				allWMRemoteProcessesArraylist.add(description);
			}
		}

		ProcessDescription[] allProcesses = new ProcessDescription[allProcessesArrayList
				.size()];
		allProcessesArrayList.toArray(allProcesses);

		ProcessDescription[] allWMLocalProcesses = new ProcessDescription[allWMLocalProcessesArraylist
				.size()];
		allWMLocalProcessesArraylist.toArray(allWMLocalProcesses);

		ProcessDescription[] allWMRemoteProcesses = new ProcessDescription[allWMRemoteProcessesArraylist
				.size()];
		allWMRemoteProcessesArraylist.toArray(allWMRemoteProcesses);

		ProcessDescription[] goalDrivenProcesses = new ProcessDescription[m_managedProcesses
				.size()];
		m_managedProcesses.toArray(goalDrivenProcesses);

		// goal drive processes plus the task manager
		ProcessDescription[] goalDrivenProcessesPlusTM = new ProcessDescription[m_managedProcesses
				.size() + 1];
		m_managedProcesses.toArray(goalDrivenProcessesPlusTM);
		goalDrivenProcessesPlusTM[goalDrivenProcessesPlusTM.length - 1] = m_taskManager;

		ArrayList<ProcessDescription> allWMLocalGoalDrivenProcessesArraylist = new ArrayList<ProcessDescription>();
		ArrayList<ProcessDescription> allWMRemoteGoalDrivenProcessesArraylist = new ArrayList<ProcessDescription>();

		for (ProcessDescription description : goalDrivenProcessesPlusTM) {
			if (description.m_language == m_workingMemory.m_language
					&& description.m_hostName
							.equals(m_workingMemory.m_hostName)) {
				allWMLocalGoalDrivenProcessesArraylist.add(description);
			} else {
				allWMRemoteGoalDrivenProcessesArraylist.add(description);
			}
		}

		ProcessDescription[] allWMRemoteGoalDrivenProcesses = new ProcessDescription[allWMRemoteGoalDrivenProcessesArraylist
				.size()];
		allWMRemoteGoalDrivenProcessesArraylist
				.toArray(allWMRemoteGoalDrivenProcesses);

		ProcessDescription[] allWMLocalGoalDrivenProcesses = new ProcessDescription[allWMLocalGoalDrivenProcessesArraylist
				.size()];
		allWMLocalGoalDrivenProcessesArraylist
				.toArray(allWMLocalGoalDrivenProcesses);

		if (allWMRemoteProcesses.length > 0) {

			// System.out.println("adding " +
			// allWMRemoteProcesses.length + "wm remote processes");

			// connection used to push into working memory
			connections.add(new ProcessConnection(allWMRemoteProcesses,
					new ProcessDescription[] { m_workingMemory },
					FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
							.typeName(WorkingMemoryEntry.class), m_said + ":"
							+ "Remote SA Working Memory Input"));
		}

		if (allWMLocalProcesses.length > 0) {

			// System.out.println("adding " + allWMLocalProcesses.length
			// + "wm loca processes");

			connections.add(new ProcessConnection(allWMLocalProcesses,
					new ProcessDescription[] { m_workingMemory },
					FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
							.typeName(CASTWorkingMemoryEntry.class), m_said
							+ ":" + "Local SA Working Memory Input"));
		}

		if (goalDrivenProcessesPlusTM.length > 0) {
			// connection used to signal change in working memory...
			// same language same machine
			connections.add(new ProcessConnection(
					new ProcessDescription[] { m_workingMemory },
					goalDrivenProcessesPlusTM,
					FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
							.typeName(WorkingMemoryChange.class), m_said + ":"
							+ "SA Working Memory Change Broadcast Local"));
		}

		// connection used to signal change in working memory
		connections.add(new ProcessConnection(goalDrivenProcessesPlusTM,
				new ProcessDescription[] { m_workingMemory },
				FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
						.typeName(WorkingMemoryChangeFilter.class), m_said
						+ ":" + "SA Working Memory Change Filters"));

		connections.add(new ProcessConnection(goalDrivenProcessesPlusTM,
				new ProcessDescription[] { m_workingMemory },
				FrameworkConnectionType.PULL_CONNECTION, "int", m_said + ":"
						+ "SA WM Overwrite Count"));

		connections.add(new ProcessConnection(allProcesses,
				new ProcessDescription[] { m_workingMemory },
				FrameworkConnectionType.PULL_CONNECTION, "bool", m_said + ":"
						+ "SA WM Exists"));

		// connections for locking and status
		connections.add(new ProcessConnection(allProcesses,
				new ProcessDescription[] { m_workingMemory },
				FrameworkConnectionType.PULL_CONNECTION, CASTUtils
						.typeName(WorkingMemoryPermissions.class), m_said + ":"
						+ "SA WM Locking"));

		// connection used to pull from working memory
		if (allWMRemoteGoalDrivenProcesses.length > 0) {
			connections.add(new ProcessConnection(
					allWMRemoteGoalDrivenProcesses,
					new ProcessDescription[] { m_workingMemory },
					FrameworkConnectionType.PULL_CONNECTION, CASTUtils
							.typeName(WorkingMemoryEntry[].class), m_said + ":"
							+ "Remote SA Working Memory Output"));
		}

		if (allWMLocalGoalDrivenProcesses.length > 0) {
			// connections.add(new ProcessConnection(
			// allWMLocalGoalDrivenProcesses,
			// new ProcessDescription[] { m_workingMemory },
			// FrameworkConnectionType.PULL_CONNECTION, CASTUtils
			// .typeName(CASTWorkingMemoryEntry[].class), m_said
			// + ":" + "Local SA Working Memory Output"));

			// HACK local representations are not easy to arrange
			connections.add(new ProcessConnection(
					allWMLocalGoalDrivenProcesses,
					new ProcessDescription[] { m_workingMemory },
					FrameworkConnectionType.PULL_CONNECTION,
					"cast::CASTWorkingMemoryEntryList", m_said + ":"
							+ "Local SA Working Memory Output"));

		}

		if (goalDrivenProcesses.length > 0) {
			// connection used to push i-p task proposals into sa goal
			// manager
			connections.add(new ProcessConnection(goalDrivenProcesses,
					new ProcessDescription[] { m_taskManager },
					FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
							.typeName(InformationProcessingTask.class), m_said
							+ ":" + "SA Goal Manager Proposal Input"));

			// connection used to push i-p task retractions into sa goal
			// manager
			connections.add(new ProcessConnection(goalDrivenProcesses,
					new ProcessDescription[] { m_taskManager },
					FrameworkConnectionType.PUSH_CONNECTION, "string", m_said
							+ ":" + "SA Goal Manager Retraction Input"));

			// connection used to push i-p task registrations into sa
			// goal
			// manager
			connections.add(new ProcessConnection(goalDrivenProcesses,
					new ProcessDescription[] { m_taskManager },
					FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
							.typeName(TaskDescription[].class), m_said + ":"
							+ "SA Goal Manager Registration Input"));

			// connection used to push into working memory
			connections.add(new ProcessConnection(
					new ProcessDescription[] { m_taskManager },
					goalDrivenProcesses,
					FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
							.typeName(TaskManagementResult.class), m_said + ":"
							+ "SA Goal Manager Output"));

			// connection used to inform sagm of processing results
			connections.add(new ProcessConnection(goalDrivenProcesses,
					new ProcessDescription[] { m_taskManager },
					FrameworkConnectionType.PUSH_CONNECTION, CASTUtils
							.typeName(TaskResult.class), m_said + ":"
							+ "SA Goal Manager PR Input"));
		}
		return connections;

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return "subarchitecture config: " + m_said;
	}

	public String getHost() {
		return m_host;
	}

	public void setHost(String _host) {
		m_host = _host;
	}

	/**
	 * @return
	 */
	public String getName() {
		return m_said;
	}

	private void addConfigurationInformation(Properties _props,
			ProcessDescription _desc) {
		_desc.m_configuration = extendConfigMap(_desc.m_configuration, _props);
	}

	private ProcessConfigurationMap extendConfigMap(
			ProcessConfigurationMap _configuration, Properties _props) {

		int oldSize = _configuration.m_keys.length;
		int newSize = _props.size() + oldSize;

		ProcessConfigurationMap newMap = new ProcessConfigurationMap(
				new String[newSize], new String[newSize]);

		System.arraycopy(_configuration.m_keys, 0, newMap.m_keys, 0, oldSize);
		System.arraycopy(_configuration.m_values, 0, newMap.m_values, 0,
				oldSize);

		Iterator<Object> keys = _props.keySet().iterator();
		for (int i = oldSize; i < newSize; i++) {
			newMap.m_keys[i] = (String) keys.next();
			newMap.m_values[i] = _props.getProperty(newMap.m_keys[i]);
		}

		return newMap;
	}

	public void addConfigurationInformation(Properties _props) {
		addConfigurationInformation(_props, m_taskManager);
		addConfigurationInformation(_props, m_workingMemory);
		for (ProcessDescription desc : m_unmanagedProcesses) {
			addConfigurationInformation(_props, desc);
		}
		for (ProcessDescription desc : m_managedProcesses) {
			addConfigurationInformation(_props, desc);
		}
	}

}
