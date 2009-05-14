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

import java.util.ArrayList;
import java.util.Iterator;

/**
 * A Java reinterpretation of the connection scheme for balt
 * processes.
 * 
 * @author nah
 */
public class ConnectionGraph {

    private ArrayList<ConnectionDescription> m_connections;

    /**
     * Create a new, empty graph.
     */
    public ConnectionGraph() {
        m_connections = new ArrayList<ConnectionDescription>();
    }

    /**
     * Create a new graph containing the given connections.
     * 
     * @param _descArray
     *            An array of connections to be added to the connection
     *            graph.
     */
    public ConnectionGraph(ConnectionDescription[] _descArray) {
        this();
        for (int i = 0; i < _descArray.length; i++) {
            m_connections.add(_descArray[i]);
        }
    }

    /**
     * Add a connection to the graph.
     * 
     * @param _cg The connection to be added.
     */
    public void addConnection(ConnectionDescription _cg) {
        m_connections.add(_cg);
    }

    /**
     * Get an iterator for the connections in the graph.
     * 
     * @return The connection iterator.
     */
    public Iterator<ConnectionDescription> iterator() {
        return m_connections.iterator();
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString() {
        String s = "Connection graph: " + m_connections.size();
	        for (Iterator<ConnectionDescription> iter = m_connections
	   .iterator(); iter.hasNext();) {
            s += "\n" + iter.next();
        }
        return s;
    }

}
