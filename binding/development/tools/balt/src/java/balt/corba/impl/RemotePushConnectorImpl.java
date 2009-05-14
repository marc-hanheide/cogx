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
package balt.corba.impl;

import org.omg.CORBA.Any;

import balt.corba.autogen.RemoteConnectors.RemotePushConnectorPOA;
import balt.corba.autogen.RemoteConnectors.RemotePushReceiver;
import balt.core.connectors.push.nonprimitive.impl.DataPair;
import balt.core.connectors.push.nonprimitive.impl.GenericPushConnectorRunnable;

public class RemotePushConnectorImpl extends RemotePushConnectorPOA {

    private static class RemotePushConnectorRunnable
            extends
                GenericPushConnectorRunnable<RemotePushReceiver, DataPair<Any>> {

        /**
         * 
         */
        public RemotePushConnectorRunnable() {
//            super(RemotePushReceiver.class, Any.class);
            super();
        }

        /*
         * (non-Javadoc)
         * 
         * @see balt.core.connectors.push.nonprimitive.impl.GenericPushConnectorRunnable#sendData(java.lang.Object,
         *      balt.core.connectors.push.nonprimitive.impl.DataPair)
         */
        @Override
        protected void sendData(RemotePushReceiver _receiver,
                DataPair<Any> _send) {
//            System.out
//                .println("RemotePushConnectorRunnable.sendData()");
            _receiver.receivePushData(_send.getSrc(), _send.getData());
        }


    }

    private RemotePushConnectorRunnable m_sendRunnable;
    private Thread m_sendThread;

    /**
     * 
     */
    public RemotePushConnectorImpl() {
        m_sendRunnable = new RemotePushConnectorRunnable();
        m_sendThread = new Thread(m_sendRunnable);
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.corba.autogen.RemoteConnectors.RemotePushConnectorOperations#push(java.lang.String,
     *      org.omg.CORBA.Any)
     */
    public void push(String _src, Any _data) {
        m_sendRunnable.queue(new DataPair<Any>(_src, _data));
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.prototypes.sandbox.corba.RemoteConnectors.RemotePushConnectorOperations#registerPushReceiver(balt.prototypes.sandbox.corba.RemoteConnectors.RemotePushReceiver)
     */
    public void registerPushReceiver(RemotePushReceiver _pr) {
//         System.out.println("RemotePushConnectorImpl.registerPushReceiver()");
        if (!m_sendRunnable.isRunning()) {
            // System.out.println("D OIT!");
            m_sendRunnable.start();
            m_sendThread.start();
        }

        m_sendRunnable.registerPushReceiver(_pr);
    }

    public void stopConnector() {
        m_sendRunnable.stop();
        try {
            m_sendThread.join();
        }
        catch (InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    /* (non-Javadoc)
     * @see balt.corba.autogen.RemoteConnectors.RemotePushConnectorOperations#flush()
     */
    public void flush() {
//        System.out.println("FLUSH FLUSH FLUSH");
        m_sendRunnable.flush();
    }
}