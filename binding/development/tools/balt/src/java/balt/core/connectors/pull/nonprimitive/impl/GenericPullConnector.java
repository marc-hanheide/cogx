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
package balt.core.connectors.pull.nonprimitive.impl;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.pull.PullReceiver;
import balt.core.data.FrameworkQuery;
import balt.management.FrameworkUtils;


public class GenericPullConnector<R, D> {

    private R m_output;
    private Method m_pullMethod;

    /**
     * Default constructor. Creates a list to hold any receivers.
     */
    public GenericPullConnector(Class<R> _recvClass, Class<D> _dataClass) {
        try {
            findReceivePullMethod(_recvClass, _dataClass);
        }
        catch (FrameworkDataTranslatorException e) {
            System.err.println(e.getLocalizedMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.prototypes.connectors.FrameworkPullConnector#pull(balt.prototypes.data.FrameworkQuery)
     */
    @SuppressWarnings("unchecked")
    public D pull(FrameworkQuery _query)
            throws FrameworkConnectionException {
        if (m_output != null) {
            try {
                return (D) m_pullMethod.invoke(m_output, new Object[] {
                    _query
                });
            }
            catch (IllegalArgumentException e) {
                e.printStackTrace();
                throw new FrameworkConnectionException(e);
            }
            catch (IllegalAccessException e) {
                e.printStackTrace();
                throw new FrameworkConnectionException(e);
            }
            catch (InvocationTargetException e) {
                e.printStackTrace();
                throw new FrameworkConnectionException(e);
            }
        }
        throw new FrameworkConnectionException(
            "m_output not set. have you called setPullReceiver?");

    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.prototypes.connectors.FrameworkPullConnector#setPullReceiver(balt.prototypes.connectors.PullReceiver)
     */
    public void setPullReceiver(R _pr) {
        m_output = _pr;
    }

    private void findReceivePullMethod(Class<? extends R> _recvClass,
            Class<? extends D> _dataClass)
            throws FrameworkDataTranslatorException {

        m_pullMethod = FrameworkUtils.findReturningMethodPrefix(
            _recvClass, _dataClass, PullReceiver.PULL_RECEIVER_METHOD,
            1);

        if (m_pullMethod == null) {
            throw new FrameworkDataTranslatorException(
                "Unable to find method: "
                    + PullReceiver.PULL_RECEIVER_METHOD + " returning "
                    + _dataClass + "in class " + _recvClass);
        }

    }

    // public static void main(String[] args) throws
    // FrameworkConnectionException {
    // GenericPullConnector<StringPullReceiver, String> gpc = new
    // GenericPullConnector<StringPullReceiver, String>(
    // StringPullReceiver.class, String.class);
    //        
    // PullReceiverProcess prp = new PullReceiverProcess("pull
    // receiver");
    //        
    // gpc.setPullReceiver(prp);
    // System.out.println(gpc.pull(new FrameworkQuery("","")));
    //        
    // }
    
    public void stopConnector() {
//        System.out.println("stop me, oh oh oh stop me");
    }


}