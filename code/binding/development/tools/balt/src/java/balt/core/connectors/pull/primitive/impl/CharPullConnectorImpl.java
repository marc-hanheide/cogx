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
package balt.core.connectors.pull.primitive.impl;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.pull.primitive.interfaces.CharPullInterface;
import balt.core.data.FrameworkQuery;

/**
 * @author nah
 */
public class CharPullConnectorImpl implements
        CharPullInterface.CharPullConnector {

    private CharPullInterface.CharPullReceiver m_out;

    /**
     * 
     */
    public CharPullConnectorImpl() {
        super();
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.pull.PullConnectorOut.CharPullConnectorOut#pull(balt.core.data.FrameworkQuery)
     */
    public char pull(FrameworkQuery _query)
            throws FrameworkConnectionException {
        return m_out.receivePullQueryChar(_query);
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.pull.PullConnectorRegister.CharPullConnectorRegister#setPullReceiver(balt.core.connectors.pull.PullReceiver.CharPullReceiver)
     */
    public void setPullReceiver(CharPullInterface.CharPullReceiver _pr) {
        m_out = _pr;
    }

    /* (non-Javadoc)
     * @see balt.core.connectors.FrameworkConnector#stopConnector()
     */
    public void stopConnector() {
        // TODO Auto-generated method stub
        
    }

}
