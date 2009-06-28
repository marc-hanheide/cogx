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
import java.util.Map;

import Ice.Communicator;
import Ice.Identity;
import cast.ComponentCreationException;
import cast.cdl.CPPSERVERPORT;
import cast.cdl.ComponentDescription;
import cast.cdl.ComponentLanguage;
import cast.cdl.JAVASERVERPORT;
import cast.core.CASTUtils;
import cast.interfaces.CASTComponentPrx;
import cast.interfaces.ComponentManagerPrx;
import cast.interfaces.ManagedComponentPrx;
import cast.interfaces.TaskManagerPrx;
import cast.interfaces.TimeServerPrx;
import cast.interfaces.UnmanagedComponentPrx;
import cast.interfaces.WorkingMemoryPrx;

/**
 * @author nah
 */
public class SubarchitectureProxies implements CASTConnectionConfiguration {

	private ArrayList<UnmanagedComponentPrx> m_unmanagedcomponents;

	private ArrayList<ManagedComponentPrx> m_managedcomponents;

	private TaskManagerPrx m_taskManager;

	private WorkingMemoryPrx m_workingMemory;

	private final String m_said;

	private final Communicator m_ic;

	private final ComponentManagerPrx m_manager;

	private final Map<String, TimeServerPrx> m_timeServers;

	
	void setWorkingMemory(ComponentDescription _config)
			throws ComponentCreationException {
		m_workingMemory = CASTUtils.createWorkingMemory(new Identity(
				_config.componentName, _config.className), m_ic,
				_config.hostName, serverPort(_config));
		configureComponent(_config, m_workingMemory);
	}

	void setTaskManager(ComponentDescription _config)
			throws ComponentCreationException {

		m_taskManager = CASTUtils.createTaskManager(new Identity(
				_config.componentName, _config.className), m_ic,
				_config.hostName, serverPort(_config));
		configureComponent(_config, m_taskManager);

	}

	void addManagedComponent(ComponentDescription _config)
			throws ComponentCreationException {
		ManagedComponentPrx mcp = CASTUtils.createManagedComponent(
				new Identity(_config.componentName, _config.className), m_ic,
				_config.hostName, serverPort(_config));
		configureComponent(_config, mcp);
		m_managedcomponents.add(mcp);
	}

	private int serverPort(ComponentDescription _config) {
		if (_config.language == ComponentLanguage.CPP) {
			return CPPSERVERPORT.value;
		} else {
			return JAVASERVERPORT.value;
		}
	}

	void addUnmanagedComponent(ComponentDescription _config)
			throws ComponentCreationException {
		UnmanagedComponentPrx ucp = CASTUtils.createUnmanagedComponent(
				new Identity(_config.componentName, _config.className), m_ic,
				_config.hostName, serverPort(_config));
		configureComponent(_config, ucp);
		m_unmanagedcomponents.add(ucp);
	}

	/**
	 * @param _config
	 * @param _cp
	 */
	private void configureComponent(ComponentDescription _config,
			CASTComponentPrx _cp) {

		TimeServerPrx ts = m_timeServers.get(_config.hostName);
		assert(ts != null);
		_cp.setTimeServer(ts);
	
		_cp.setComponentManager(m_manager);
		_cp.configure(_config.configuration);
	}

	/**
	 * @return the taskManager
	 */
	public TaskManagerPrx getTaskManager() {
		return m_taskManager;
	}

	/**
	 * @return the said
	 */
	public String getID() {
		return m_said;
	}

	/**
	 * @param _man
	 * @param _servers 
	 * @throws ComponentCreationException
	 * 
	 */

	public SubarchitectureProxies(SubarchitectureConfiguration _config,
			ComponentManagerPrx _man, Map<String, TimeServerPrx> _servers, Communicator _communicator)
			throws ComponentCreationException {
		m_said = _config.getID();
		m_manager = _man;
		m_timeServers = _servers;
		m_managedcomponents = new ArrayList<ManagedComponentPrx>(1);
		m_unmanagedcomponents = new ArrayList<UnmanagedComponentPrx>(1);
		m_ic = _communicator;

		setWorkingMemory(_config.getWorkingMemoryConfig());
		setTaskManager(_config.getTaskManagerConfig());
		for (ComponentDescription config : _config.getManagedComponents()) {
			addManagedComponent(config);
		}
		for (ComponentDescription config : _config.getUnmanagedComponents()) {
			addUnmanagedComponent(config);
		}
	}

	/**
	 * @return the dataDrivencomponent
	 */
	public ArrayList<UnmanagedComponentPrx> getUnmanagedComponents() {
		return m_unmanagedcomponents;
	}

	/**
	 * @return the goalDrivencomponent
	 */
	public ArrayList<ManagedComponentPrx> getManagedComponents() {
		return m_managedcomponents;
	}

	public WorkingMemoryPrx getWorkingMemory() {
		return m_workingMemory;
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

}
