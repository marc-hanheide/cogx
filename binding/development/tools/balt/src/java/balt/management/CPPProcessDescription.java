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

public class CPPProcessDescription extends ManagedProcessDescription {

    private String m_processClass;

    /**
     * @param _processClass
     * @param _processConfig
     */
    public CPPProcessDescription(String _processName,
            String _processClass, boolean _local) {
        super(_processName, _local, null);
        m_processClass = _processClass;
    }

    /**
     * @param _processClass
     * @param _processConfig
     */
    public CPPProcessDescription(String _processName,
            String _processClass, boolean _local,
            Properties _processConfig) {
        super(_processName, _local, _processConfig);
        m_processClass = _processClass;
    }

    /**
     * @return Returns the processClass.
     */
    public String getProcessClass() {
        return m_processClass;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#equals(java.lang.Object)
     */
    @Override
    public boolean equals(Object _obj) {
        if (_obj instanceof CPPProcessDescription) {
            CPPProcessDescription pd = (CPPProcessDescription) _obj;
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
        return "CPP: " + m_processName + ", local = " + isLocal();
    }

}
