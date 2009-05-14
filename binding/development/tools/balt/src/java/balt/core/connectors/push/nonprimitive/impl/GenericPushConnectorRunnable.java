/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * Copyright (C) 2006-2007 Nick Hawes This library is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * Lesser General Public License as published by the Free Software
 * Foundation; either version 2.1 of the License, or (at your option)
 * any later version. This library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU Lesser General Public License for more details. You should
 * have received a copy of the GNU Lesser General Public License along
 * with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * 
 */
package balt.core.connectors.push.nonprimitive.impl;

import java.util.Vector;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Semaphore;

import balt.core.connectors.ControlledRunnable;

public abstract class GenericPushConnectorRunnable<T, D>
        extends
            ControlledRunnable {

    // these are synchronised ;)
    protected Vector<T> m_receivers;

    // this is synchronised by us
    private ConcurrentLinkedQueue<D> m_dataList;

    // this should have permits when there is something in the list
    private Semaphore m_listContentsSemaphore;

    // lock to stop other things happening during flush
    private Semaphore m_flushLock;
    private Object m_listIsEmptySemaphore;

    private boolean m_isSending;

    /**
     * 
     */
    public GenericPushConnectorRunnable() {
        super();
        m_receivers = new Vector<T>(1);
        m_dataList = new ConcurrentLinkedQueue<D>();
        m_listContentsSemaphore = new Semaphore(0, true);
        m_flushLock = new Semaphore(1, true);

        m_isSending = true; // safer to assume that something is
                            // happening
        m_listIsEmptySemaphore = new Object();

    }

    public void registerPushReceiver(T _pr) {
        m_receivers.add(_pr);
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.connectors.ControlledRunnable#stop()
     */
    @Override
    public void stop() {
        super.stop();

        m_dataList.clear();

        // synchronized (this) {
        // notifyAll();
        // }

        // try to free all waiting threads
        m_listContentsSemaphore.release(m_listContentsSemaphore
            .getQueueLength());
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Runnable#run()
     */
    public void run() {

        D m_toSend = null;
        while (isRunning()) {

            while (isRunning() && !m_dataList.isEmpty()) {

                m_toSend = m_dataList.poll();
                synchronized (m_receivers) {
                    for (T receiver : m_receivers) {
                        try {
                            sendData(receiver, m_toSend);
                        }
                        catch (RuntimeException e) {
                            e.printStackTrace();
                            System.exit(1);
                        }

                    }
                }

            }

            // we're not sending any more
            m_isSending = false;

            synchronized (m_listIsEmptySemaphore) {
                m_listIsEmptySemaphore.notifyAll();
            }

            try {
                m_listContentsSemaphore.acquire();
            }
            catch (InterruptedException e) {
                e.printStackTrace();
                System.exit(1);
            }

            m_isSending = true;

            // synchronized (this) {
            // while (isRunning() && m_dataList.isEmpty()) {
            // try {
            // wait();
            // }
            // catch (InterruptedException e) {
            // e.printStackTrace();
            // System.exit(1);
            // }
            // }
            // }
        }
    }

    /**
     * @param _receiver
     * @param _send
     */
    protected abstract void sendData(T _receiver, D _send);

    /**
     * @param _data
     */
    @SuppressWarnings("unchecked")
    public void queue(D _data) {

        try {
            m_flushLock.acquire();
        }
        catch (InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }

        // synchronized (this) {
        // it's important to sync both calls, as this prevents
        // the addition happening before a wait
        m_dataList.add(_data);
        m_listContentsSemaphore.release();
        // notifyAll();
        // }

        m_flushLock.release();
    }

    /**
     * 
     */
    public void flush() {
        // System.out.println("flushing");
        try {
            m_flushLock.acquire();
        }
        catch (InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
        synchronized (m_listIsEmptySemaphore) {
            while (!m_dataList.isEmpty() || m_isSending) {

                try {
                    m_listIsEmptySemaphore.wait();
                }
                catch (InterruptedException e) {
                    e.printStackTrace();
                    System.exit(1);
                }
            }
        }

        assert (m_dataList.isEmpty());
        // System.out.println("about to release flushlock");
        m_flushLock.release();
    }

}