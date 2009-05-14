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
package balt.corba.connectors.push.primitive;

import balt.corba.autogen.RemoteConnectors.RemotePushConnector;
import balt.corba.autogen.RemoteConnectors.RemotePushSenderPOA;
import balt.corba.data.RemoteDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.core.connectors.push.primitive.interfaces.IntPushInterface.IntPushConnectorOut;

/**
 * @author nah
 */
public class IntRemotePushSenderImpl extends RemotePushSenderPOA
        implements IntPushConnectorOut {

    private RemotePushConnector m_out;

    /**
     * 
     */
    public IntRemotePushSenderImpl() {
        super();
    }


    //these are all out of date because they're not threaded
    public void flush() {
        
    }

    
    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.push.PushConnectorOut.LongPushConnectorOut#push(java.lang.String,
     *      int)
     */
    public void push(String _src, int _data) {
        try {
            m_out
                .push(_src, RemoteDataTranslator.translateToAny(_data));
        }
        catch (FrameworkDataTranslatorException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.corba.autogen.RemoteConnectors.RemotePushSenderOperations#setPushConnector(balt.corba.autogen.RemoteConnectors.RemotePushConnector)
     */
    public void setPushConnector(RemotePushConnector _out) {
        m_out = _out;
    }

}
