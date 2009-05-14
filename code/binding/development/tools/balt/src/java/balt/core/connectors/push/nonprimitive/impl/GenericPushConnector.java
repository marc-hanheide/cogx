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
package balt.core.connectors.push.nonprimitive.impl;



public class GenericPushConnector<T, D> {

    private PushConnectorRunnable<T, D> m_senderRunnable;
    private Thread m_senderThread;

    /**
     * Default constructor. Creates a list to hold any receivers.
     */
    @SuppressWarnings("unchecked")
    public GenericPushConnector(Class<T> _recvClass, Class<D> _dataClass) {
        m_senderRunnable = new PushConnectorRunnable(_recvClass,
            _dataClass);
        m_senderThread = new Thread(m_senderRunnable);
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.PushConnectorOut.StringPushConnectorOut#push(java.lang.String,
     *      java.lang.String)
     */
    @SuppressWarnings("unchecked")
    public void push(String _src, D _data) {
        m_senderRunnable.queue(new DataPair(_src, _data));
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.PushConnectorRegister.StringPushConnectorRegister#registerPushReceiver(balt.core.connectors.PushReceiver.StringPushReceiver)
     */
    public void registerPushReceiver(T _pr) {
        if (!m_senderRunnable.isRunning()) {
            m_senderRunnable.start();
            m_senderThread.start();
        }

        m_senderRunnable.registerPushReceiver(_pr);
    }

    
    
    public void flush() {
        m_senderRunnable.flush();
    }
    
    public void stopConnector() {
        m_senderRunnable.stop();
        try {
            m_senderThread.join();
        }
        catch (InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }
}