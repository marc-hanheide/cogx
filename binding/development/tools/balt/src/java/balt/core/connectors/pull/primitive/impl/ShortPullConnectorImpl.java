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
import balt.core.connectors.pull.primitive.interfaces.ShortPullInterface;
import balt.core.data.FrameworkQuery;

/**
 * @author nah
 */
public class ShortPullConnectorImpl implements
        ShortPullInterface.ShortPullConnector {

    private ShortPullInterface.ShortPullReceiver m_out;

    /**
     * 
     */
    public ShortPullConnectorImpl() {
        super();
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.pull.PullConnectorOut.ShortPullConnectorOut#pull(balt.core.data.FrameworkQuery)
     */
    public short pull(FrameworkQuery _query)
            throws FrameworkConnectionException {
        return m_out.receivePullQueryShort(_query);
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.pull.PullConnectorRegister.ShortPullConnectorRegister#setPullReceiver(balt.core.connectors.pull.PullReceiver.ShortPullReceiver)
     */
    public void setPullReceiver(ShortPullInterface.ShortPullReceiver _pr) {
        m_out = _pr;
    }

    /* (non-Javadoc)
     * @see balt.core.connectors.FrameworkConnector#stopConnector()
     */
    public void stopConnector() {
        // TODO Auto-generated method stub
        
    }

}
