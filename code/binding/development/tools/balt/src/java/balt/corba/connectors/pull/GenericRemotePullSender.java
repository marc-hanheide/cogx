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
package balt.corba.connectors.pull;

import org.omg.CORBA.Any;

import balt.corba.autogen.RemoteConnectors.RemotePullConnector;
import balt.corba.data.RemoteDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.core.connectors.FrameworkConnectionException;
import balt.core.data.FrameworkQuery;

/**
 * Worker class for remote senders.
 * 
 * @author nah
 */
public class GenericRemotePullSender<D> {

    private RemotePullConnector m_out;
    private Class<D> m_dataClass;

    /**
     * 
     */
    public GenericRemotePullSender(Class<D> _dataClass) {
        m_dataClass = _dataClass;
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.prototypes.sandbox.corba.RemoteConnectors.RemotePushSenderOperations#setPushConnector(balt.prototypes.sandbox.corba.RemoteConnectors.RemotePushConnector)
     */
    public void setPullConnector(RemotePullConnector _out) {
        m_out = _out;
    }
 
    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.PullConnectorOut#pull(balt.core.data.FrameworkQuery)
     */
    public D pull(FrameworkQuery _query)
            throws FrameworkConnectionException {
        if (m_out != null) {
            Any a = m_out.pull(_query.getSource().toString(), _query
                .getQuery());
            try {
                return RemoteDataTranslator.translateFromAny(a,
                    m_dataClass);
            }
            catch (FrameworkDataTranslatorException e) {
                throw new FrameworkConnectionException(
                    "Problem translating data", e);
            }
        }

        throw new FrameworkConnectionException(
            "RemotePullConnector not set");
    }

}
