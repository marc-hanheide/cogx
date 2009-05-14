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
package cast.ui.architecture;

import java.io.PrintStream;
import java.util.*;

import cast.configuration.ArchitectureConfiguration;
import cast.configuration.SubarchitectureConfiguration;

/**
 * A class that monitors a whole caat architecture.
 * 
 * @author nah
 */
public class ArchitectureMonitor implements
        Iterable<SubarchitectureMonitor> {

    private HashMap<String, SubarchitectureMonitor> m_subarchitectures;

    // nah: no longer used
    // private ComponentMonitor m_motiveGen;
    // private ComponentMonitor m_motiveMan;
    // private String m_motiveGenID;
    // private String m_motiveManID;

    /**
     * 
     */
    public ArchitectureMonitor(ArchitectureConfiguration _config,
            PrintStream _outStream) {
        createMonitors(_config, _outStream);
    }

    /**
     * @param _config
     * @param _stream
     */
    private void createMonitors(ArchitectureConfiguration _config,
                                PrintStream _stream) {
        m_subarchitectures =
                new HashMap<String, SubarchitectureMonitor>();

        // if (!_config.isSingleSubarchConfig()) {
        // m_motiveGenID = _config.getMotiveGenerator().m_processName;
        // m_motiveGen = new ComponentMonitor(m_motiveGenID);
        //
        // m_motiveManID = _config.getGoalManager().m_processName;
        // m_motiveMan = new ComponentMonitor(m_motiveManID);
        // }

        for (SubarchitectureConfiguration configuration : _config) {
            SubarchitectureMonitor monitor =
                    new SubarchitectureMonitor(configuration, _stream);
            m_subarchitectures.put(monitor.getID(), monitor);
        }
    }

    /**
     * @param _component
     * @param _subarchitecture
     * @return
     * @throws CASTUIException
     */
    public ComponentMonitor getComponent(String _component,
                                         String _subarchitecture)
            throws CASTUIException {

        SubarchitectureMonitor saMonitor =
                m_subarchitectures.get(_subarchitecture);

        ComponentMonitor monitor = null;

        if (saMonitor == null) {
//            if (_component.equals(m_motiveGenID)) {
//                monitor = m_motiveGen;
//            }
//            else if (_component.equals(m_motiveManID)) {
//                monitor = m_motiveMan;
//            }
        }
        else {
            monitor = saMonitor.getComponent(_component);
        }

        if (monitor == null) {
            throw new CASTUIException("monitor not found for: "
                + _component + " from sa " + _subarchitecture);
        }

        return monitor;
    }

//    /**
//     * @return the motiveGen
//     */
//    public ComponentMonitor getMotiveGenerator() {
//        return m_motiveGen;
//    }
//
//    /**
//     * @return the motiveMan
//     */
//    public ComponentMonitor getMotiveManager() {
//        return m_motiveMan;
//    }

    /**
     * @return the subarchitectures
     */
    public Collection<SubarchitectureMonitor> getSubarchitectures() {
        return m_subarchitectures.values();
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Iterable#iterator()
     */
    public Iterator<SubarchitectureMonitor> iterator() {
        return m_subarchitectures.values().iterator();
    }

}
