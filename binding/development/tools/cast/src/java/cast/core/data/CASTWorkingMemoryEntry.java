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
package cast.core.data;

import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author nah
 */
public class CASTWorkingMemoryEntry<T> {

    private String m_src;
    private WorkingMemoryOperation m_operation;
    private WorkingMemoryAddress m_address;
//    private int m_version;
    private CASTWorkingMemoryItem<T> m_item;

    public CASTWorkingMemoryEntry(String _src,
            WorkingMemoryOperation _operation,
            WorkingMemoryAddress _address,
//            int _version,
            CASTWorkingMemoryItem<T> _item) {
        m_src = _src;
        m_operation = _operation;
        m_address = _address;
//        m_version = _version;
        m_item = _item;
    }

    public WorkingMemoryAddress getAddress() {
        return m_address;
    }

    public void setAddress(WorkingMemoryAddress _address) {
        m_address = _address;
    }

    public CASTWorkingMemoryItem<T> getItem() {
        return m_item;
    }

    public void setItem(CASTWorkingMemoryItem<T> _item) {
        m_item = _item;
    }

    public WorkingMemoryOperation getOperation() {
        return m_operation;
    }

    public void setOperation(WorkingMemoryOperation _operation) {
        m_operation = _operation;
    }

    public String getSrc() {
        return m_src;
    }

    public void setSrc(String _src) {
        m_src = _src;
    }

    /* (non-Javadoc)
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString() {
        String out = "";
        out += CASTUtils.toString(getAddress()) + "\n";
        out += getSrc() + "\n";
        out += CASTUtils.toString(getOperation());
        
        return out;
    }

//	public int getVersion() {
//		return m_version;
//	}
//
//	public void setVersion(int _version) {
//		m_version = _version;
//	}
    
}
