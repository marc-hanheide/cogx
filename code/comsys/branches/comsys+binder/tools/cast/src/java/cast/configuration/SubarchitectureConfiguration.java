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
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import cast.cdl.ComponentDescription;
import cast.cdl.ComponentLanguage;
import cast.cdl.SUBARCHIDKEY;

/**
 * @author nah
 */
public class SubarchitectureConfiguration implements
		CASTConnectionConfiguration {

	private ArrayList<ComponentDescription> m_unmanagedcomponents;

	private ArrayList<ComponentDescription> m_managedcomponents;

	private ComponentDescription m_taskManager;

	private ComponentDescription m_workingMemory;

	private String m_said;

	private Map<String, String> m_subarchDefaults;

	private String m_host;

	// /All hosts used by this sa.
	private final Set<String> m_hosts;

	// if a java component is encountered this stores its hostname for later.
	// Used to get a machine to run the managing processes on.
	private String m_aJavaServer;

	public String getAJavaServer() {
		return m_aJavaServer;
	}

	/**
	 * Return all components as descriptions.
	 * 
	 * @return
	 */
	public ArrayList<ComponentDescription> getDescriptions() {
		ArrayList<ComponentDescription> descr = new ArrayList<ComponentDescription>();
		descr.add(m_taskManager);
		descr.add(m_workingMemory);
		descr.addAll(m_unmanagedcomponents);
		descr.addAll(m_managedcomponents);
		return descr;
	}

	/**
	 * @return the taskManager
	 */
	public ComponentDescription getTaskManagerConfig() {
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
		m_managedcomponents = new ArrayList<ComponentDescription>(1);
		m_unmanagedcomponents = new ArrayList<ComponentDescription>(1);

		m_subarchDefaults = new HashMap<String, String>();
		m_subarchDefaults.put(SUBARCHIDKEY.value, _said);
		m_hosts = new HashSet<String>();
	}

	/**
	 * @param _workingMemoryClass
	 * @param _lang
	 */
	public void setWorkingMemory(String _workingMemoryClass,
			ComponentLanguage _lang, String _host) {
		setWorkingMemory(_workingMemoryClass, _lang, _host,
				new HashMap<String, String>());
	}

	/**
	 * @param _workingMemoryClass
	 * @param _lang
	 */
	public void setWorkingMemory(String _workingMemoryClass,
			ComponentLanguage _lang, String _host, Map<String, String> _props) {

		if (m_aJavaServer == null && _lang == ComponentLanguage.JAVA) {
			m_aJavaServer = _host;
		}

		Map<String, String> props = new HashMap<String, String>(
				m_subarchDefaults);
		props.putAll(_props);

		m_workingMemory = new ComponentDescription(m_said + "_wm",
				_workingMemoryClass, _lang, _host, props);
		if (m_aJavaServer == null && _lang == ComponentLanguage.JAVA) {
			m_aJavaServer = _host;
		}

		addHost(_host);
		// System.out.println("props: " +
		// props.getProperty(SUBARCH_ID_KEY.value));
	}

	private void addHost(String _host) {
		m_hosts.add(_host);
	}

	/**
	 * @param _taskManagerClass
	 * @param _lang
	 */
	public void setTaskManager(String _taskManagerClass,
			ComponentLanguage _lang, String _host) {
		setTaskManager(_taskManagerClass, _lang, _host,
				new HashMap<String, String>());
	}

	public void setTaskManager(String _taskManagerClass,
			ComponentLanguage _lang, String _host, Map<String, String> _props) {

		if (m_aJavaServer == null && _lang == ComponentLanguage.JAVA) {
			m_aJavaServer = _host;
		}

		Map<String, String> props = new HashMap<String, String>(
				m_subarchDefaults);
		props.putAll(_props);

		m_taskManager = new ComponentDescription(m_said + "_tm",
				_taskManagerClass, _lang, _host, props);

		addHost(_host);

	}

	/**
	 * @return the dataDrivencomponent
	 */
	public ArrayList<ComponentDescription> getUnmanagedComponents() {
		return m_unmanagedcomponents;
	}

	/**
	 * @return the goalDrivencomponent
	 */
	public ArrayList<ComponentDescription> getManagedComponents() {
		return m_managedcomponents;
	}

	public void addUnmanagedComponent(String _name, String _class,
			ComponentLanguage _lang, String _host) {
		addUnmanagedComponent(_name, _class, _lang, _host,
				new HashMap<String, String>());
	}

	public void addUnmanagedComponent(String _name, String _class,
			ComponentLanguage _lang, String _host, Map<String, String> _props) {

		if (m_aJavaServer == null && _lang == ComponentLanguage.JAVA) {
			m_aJavaServer = _host;
		}

		Map<String, String> props = new HashMap<String, String>(
				m_subarchDefaults);
		props.putAll(_props);

		m_unmanagedcomponents.add(new ComponentDescription(_name, _class,
				_lang, _host, props));

		addHost(_host);

	}

	public void addManagedComponent(String _name, String _class,
			ComponentLanguage _lang, String _host) {
		addManagedComponent(_name, _class, _lang, _host,
				new HashMap<String, String>());
	}

	public void addManagedComponent(String _name, String _class,
			ComponentLanguage _lang, String _host, Map<String, String> _props) {

		if (m_aJavaServer == null && _lang == ComponentLanguage.JAVA) {
			m_aJavaServer = _host;
		}

		Map<String, String> props = new HashMap<String, String>(
				m_subarchDefaults);
		props.putAll(_props);

		m_managedcomponents.add(new ComponentDescription(_name, _class, _lang,
				_host, props));
		addHost(_host);

	}

	public ComponentDescription getWorkingMemoryConfig() {
		return m_workingMemory;
	}

	public Set<String> getHosts() {
		return m_hosts;
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

	private void addConfigurationInformation(Map<String, String> _props,
			ComponentDescription _desc) {
		_desc.configuration.putAll(_props);
	}

	public void addConfigurationInformation(Map<String, String> _props) {
		addConfigurationInformation(_props, m_taskManager);
		addConfigurationInformation(_props, m_workingMemory);
		for (ComponentDescription desc : m_unmanagedcomponents) {
			addConfigurationInformation(_props, desc);
		}
		for (ComponentDescription desc : m_managedcomponents) {
			addConfigurationInformation(_props, desc);
		}
	}

}
