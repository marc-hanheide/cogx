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
package balt.management;

import java.util.Properties;

/**
 * Superclass for process descriptions.
 * 
 * @author nah
 */
public class ManagedProcessDescription {

    /**
     * Boolean to determine if the process is local to the current machine.
     */
    protected boolean bIsLocal;
    
    /**
     * Configuration information for the process.
     */
    protected Properties m_processConfig;
    
    /**
     * Unique name for the process.
     */
    protected String m_processName;


    /**
     * 
     * Create a basic process description,
     * 
     * @param _processName The name of the process.
     * @param _local Whether the process is local or not.
     * @param _processConfig Configuration for the process.
     */
    public ManagedProcessDescription(String _processName,
            boolean _local, Properties _processConfig) {
//        System.out
//            .println("ManagedProcessDescription.ManagedProcessDescription()");
//        System.out
//            .println(_processConfig);
        
        m_processName = _processName;
        m_processConfig = _processConfig;
        bIsLocal = _local;
    }


    /**
     * Set the configuration information for this process.
     * 
     * @param processConfig
     *            The processConfig to set.
     */
    public void setProcessConfiguration(
            Properties processConfig) {
        m_processConfig = processConfig;
    }

    /**
     * Get the configuration for this process.
     * 
     * @return Returns the processConfig.
     */
    public Properties getProcessConfiguration() {
        return m_processConfig;
    }

    /**
     * Get the name of this process.
     * @return The name of the process.
     */
    public String getProcessName() {
        return m_processName;
    }

    /**
     * Check whether this process is local or not.
     * 
     * @return True if the process is local.
     */
    public boolean isLocal() {
        return bIsLocal;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#hashCode()
     */
    @Override
    public int hashCode() {
        return m_processName.hashCode();
    }

}