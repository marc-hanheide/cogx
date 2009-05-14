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
package balt.corba.connectors.pull.primitive;

import balt.corba.autogen.RemoteConnectors.RemotePullConnector;
import balt.corba.autogen.RemoteConnectors.RemotePullSenderPOA;
import balt.corba.data.RemoteDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.pull.primitive.interfaces.CharPullInterface;
import balt.core.data.FrameworkQuery;

/**
 * @author nah
 */
public class CharRemotePullSenderImpl extends RemotePullSenderPOA
        implements CharPullInterface.CharPullConnectorOut {

    private RemotePullConnector m_out;

    /**
     * 
     */
    public CharRemotePullSenderImpl() {
        super();
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.pull.PullConnectorOut.CharPullConnectorOut#pull(balt.core.data.FrameworkQuery)
     */
    public char pull(FrameworkQuery _query)
            throws FrameworkConnectionException {
        if (m_out != null) {
            try {
                return RemoteDataTranslator.translateCharFromAny(m_out
                    .pull(_query.getSource().toString(), _query
                        .getQuery()));
            }
            catch (FrameworkDataTranslatorException e) {
                throw new FrameworkConnectionException(e);
            }
        }
        throw new FrameworkConnectionException(
            "RemotePullConnector not set");
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.corba.autogen.RemoteConnectors.RemotePullSenderOperations#setPullConnector(balt.corba.autogen.RemoteConnectors.RemotePullConnector)
     */
    public void setPullConnector(RemotePullConnector _out) {
        m_out = _out;
    }

    /* (non-Javadoc)
     * @see balt.core.connectors.FrameworkConnector#stopConnector()
     */
    public void stopConnector() {
        // TODO Auto-generated method stub
        
    }

}
