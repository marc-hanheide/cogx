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


package balt.core.connectors.push;

/**
 * Interface to define a connector that can be used for push output from
 * a process. This is separated from the PushConnectorRegister interface
 * because the remote connectors do not need both aspects of the
 * functionality.
 * 
 * @author nah
 */
public interface PushConnectorOut {

    public static final String PUSH_CONNECTOR_OUT_METHOD = "push";

    /**
     * Flush the connector.
     */
    void flush();
}