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

package balt.core.data;

public class FrameworkLocalData<T> {

    protected FrameworkProcessIdentifier m_source;

    protected T m_data;

    // public FrameworkLocalData() {
    // m_source = null;
    // m_data = null;
    // }

    public FrameworkLocalData(String _source, T _data) {
        m_source = new FrameworkProcessIdentifier(_source);
        m_data = _data;
    }

    public FrameworkLocalData(FrameworkProcessIdentifier _source,
            T _data) {
        m_source = _source;
        m_data = _data;
    }

    public T getData() {
        return m_data;
    }

    public FrameworkProcessIdentifier getSource() {
        return m_source;
    }

    // public void setData(T _data) {
    // m_data = _data;
    // }
    //
    // public void setSource(String _src) {
    // m_source = new FrameworkProcessIdentifier(_src);
    // }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#equals(java.lang.Object)
     */
    @Override
    public boolean equals(Object obj) {
        if (obj instanceof FrameworkLocalData) {
            FrameworkLocalData fd = (FrameworkLocalData) obj;
            return fd.m_data.equals(m_data)
                && fd.m_source.equals(m_source);
        }
        else {
            // System.out.println("not instanceof");
            return false;
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString() {
        return "[FrameworkData source=\"" + m_source.toString()
            + "\" data=\"" + m_data.toString() + "\"]";
    }

}
