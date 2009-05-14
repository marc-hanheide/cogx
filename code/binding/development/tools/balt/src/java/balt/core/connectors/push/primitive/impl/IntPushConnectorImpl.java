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

import balt.core.connectors.push.primitive.interfaces.IntPushInterface;


/**
 * @author nah
 */
public class IntPushConnectorImpl implements
        IntPushInterface.IntPushConnector {

    private ArrayList<IntPushInterface.IntPushReceiver> output;

    /**
     * 
     */
    public IntPushConnectorImpl() {
        output = new ArrayList<IntPushInterface.IntPushReceiver>(1);
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.push.PushConnectorOut.IntPushConnectorOut#push(java.lang.String,
     *      int)
     */
    public void push(String _src, int _data) {
        for (IntPushInterface.IntPushReceiver pr : output) {
            pr.receivePushData(_src, _data);
        }
    }


    //these are all out of date because they're not threaded
    public void flush() {
        
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.push.PushConnectorRegister.IntSeqPushConnectorRegister#registerPushReceiver(balt.core.connectors.push.PushReceiver.IntSeqPushReceiver)
     */
    public void registerPushReceiver(
            IntPushInterface.IntPushReceiver _pr) {
        output.add(_pr);
    }

    /* (non-Javadoc)
     * @see balt.core.connectors.FrameworkConnector#stopConnector()
     */
    public void stopConnector() {
        // TODO Auto-generated method stub
        
    }

}
