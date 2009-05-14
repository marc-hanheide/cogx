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

package balt.management;

import java.util.Properties;

/**
 * Class to represent a Java process in the balt.
 * 
 * @author nah
 */
public class JavaProcessDescription extends ManagedProcessDescription {

    /**
     * The class for the process.
     */
    private Class m_processClass;

    /**
     * 
     * Create a new description for a Java process.
     * 
     * @param _processName The name of the process.
     * @param _processClass The class of the process.
     * @param _local Whether this process is local.
     * @param _processConfig Configuration information for the process.
     */
    public JavaProcessDescription(String _processName,
            Class _processClass, boolean _local, Properties _processConfig) {
        super(_processName, _local, _processConfig);
        m_processClass = _processClass;
    }

    /**
     * Get the class of the process.
     * 
     * @return Returns the processClass.
     */
    public Class getProcessClass() {
        return m_processClass;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#equals(java.lang.Object)
     */
    @Override
    public boolean equals(Object _obj) {
        if (_obj instanceof JavaProcessDescription) {
            JavaProcessDescription pd = (JavaProcessDescription) _obj;
            return pd.getProcessName().equals(m_processName);
        }
        return false;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString() {
        return "Java: " + m_processName + ", local = " + isLocal();
    }

}
