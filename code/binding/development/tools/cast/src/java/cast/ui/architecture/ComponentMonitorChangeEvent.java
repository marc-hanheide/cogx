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

import java.util.EventObject;

import cast.cdl.ui.ComponentEventType;

/**
 * @author nah
 */
public class ComponentMonitorChangeEvent extends EventObject {

    private ComponentEventType m_changeType;

    /**
     * 
     */
    private static final long serialVersionUID = -4424014569147917361L;

    /**
     * @param _source
     */
    public ComponentMonitorChangeEvent(ComponentMonitor _source,
            ComponentEventType _changeType) {
        super(_source);
        m_changeType = _changeType;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.util.EventObject#getSource()
     */
    @Override
    public ComponentMonitor getSource() {
        // TODO Auto-generated method stub
        return (ComponentMonitor) super.getSource();
    }

    
    /**
     * @return the changeType
     */
    public ComponentEventType getChangeType() {
        return m_changeType;
    }
    
}
