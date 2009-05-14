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

import java.lang.reflect.Method;

import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.core.connectors.push.PushReceiver;

class PushConnectorRunnable<T, D> extends GenericPushConnectorRunnable<T, DataPair<D>> {

    private Method m_recvMethod;

    /**
     * 
     */
    public PushConnectorRunnable(Class<T> _recvClass, Class<D> _dataClass) {

        super();
        
        // System.out.println("PushSenderRunnable.PushSenderRunnable()");
        // System.out.println(_dataClass);

        try {
            findReceiveMethod(_recvClass, _dataClass);
        }
        catch (FrameworkDataTranslatorException e) {
            System.err
                .println("unable to find the correct receiver method in class: "
                    + _recvClass);
            System.err.println("need: " + "receivePushData(String,"
                + _dataClass + ")");
            e.printStackTrace();
            System.exit(1);
        }

    }

    private void findReceiveMethod(Class<? extends T> _recvClass,
            Class<? extends D> _dataClass)
            throws FrameworkDataTranslatorException {

        try {
            m_recvMethod = _recvClass.getMethod(
                PushReceiver.PUSH_RECEIVER_METHOD, new Class[] {
                    String.class, _dataClass
                });
        }
        catch (SecurityException e) {
            throw new FrameworkDataTranslatorException(e);
        }
        catch (NoSuchMethodException e) {
            throw new FrameworkDataTranslatorException(e);
        }
        catch (IllegalArgumentException e) {
            throw new FrameworkDataTranslatorException(e);
        }

    }

    /**
     * @param _receiver
     * @param _send
     */
    protected void sendData(T _receiver, DataPair<D> _send) {
        try {
            m_recvMethod.invoke(_receiver, _send.m_src, _send.m_data);
        }
        catch (Exception e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

  


}