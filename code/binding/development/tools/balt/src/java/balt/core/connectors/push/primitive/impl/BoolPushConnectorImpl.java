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
package balt.core.connectors.push.primitive.impl;

import java.util.ArrayList;

import balt.core.connectors.push.primitive.interfaces.BoolPushInterface;


/**
 * @author nah
 */
public class BoolPushConnectorImpl implements
        BoolPushInterface.BoolPushConnector {

    private ArrayList<BoolPushInterface.BoolPushReceiver> output;

    /**
     * 
     */
    public BoolPushConnectorImpl() {
        output = new ArrayList<BoolPushInterface.BoolPushReceiver>(1);
    }


  

    
    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.push.PushConnectorOut.BoolPushConnectorOut#push(java.lang.String,
     *      int)
     */
    public void push(String _src, boolean _data) {
        for (BoolPushInterface.BoolPushReceiver pr : output) {
            pr.receivePushData(_src, _data);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.push.PushConnectorRegister.BoolSeqPushConnectorRegister#registerPushReceiver(balt.core.connectors.push.PushReceiver.BoolSeqPushReceiver)
     */
    public void registerPushReceiver(
            BoolPushInterface.BoolPushReceiver _pr) {
        output.add(_pr);
    }

    /* (non-Javadoc)
     * @see balt.core.connectors.FrameworkConnector#stopConnector()
     */
    public void stopConnector() {
        // TODO Auto-generated method stub
        
    }

    //these are all out of date because they're not threaded
    public void flush() {
        
    }
    
}
