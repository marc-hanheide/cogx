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

import cast.cdl.SUBARCHIDKEY;

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
		m_archDefaults.setProperty(SUBARCHIDKEY.value, "general.arch");

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
	 * @see java.lang.Iterable#iterator()
	 */
	public Iterator<SubarchitectureConfiguration> iterator() {
		return m_subarchitectures.iterator();
	}

}
