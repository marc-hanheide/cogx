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

package balt.core.connectors;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;

import balt.core.autocode.ConnectorGenerator;
import balt.core.connectors.pull.*;
import balt.core.connectors.pull.nonprimitive.impl.GenericPullConnector;
import balt.core.connectors.pull.primitive.impl.*;
import balt.core.connectors.push.*;
import balt.core.connectors.push.nonprimitive.impl.GenericPushConnector;
import balt.core.connectors.push.primitive.impl.*;
import balt.management.FrameworkUtils;

/**
 * A class that wraps up the local connection code. This is used to
 * connect local Java processes in any way allowable. It also provides
 * protected implementations of local push and pull connectors. \todo
 * Remove static-ness to mirror C++ structure and make having members
 * more sensible. \todo Different connections types and counts.
 * 
 * @author nah
 */
public class LocalConnectionManager {

    // keep connection objects hidden to stop people using them
    // directly!

    private static ArrayList<FrameworkConnector> m_connections;

    static {
        m_connections = new ArrayList<FrameworkConnector>();
    }

    private static <D, R, C> PullConnector generatePullConnector(
            Class<D> _data, Class<R> _recvClass,
            Class<C> _connectorClass)
            throws ConnectionCreationException {

        if (_data.isPrimitive()) {
            if (_data.equals(int.class)) {
                return new IntPullConnectorImpl();
            }
            else if (_data.equals(long.class)) {
                return new LongPullConnectorImpl();
            }
            else if (_data.equals(short.class)) {
                return new ShortPullConnectorImpl();
            }
            else if (_data.equals(double.class)) {
                return new DoublePullConnectorImpl();
            }
            else if (_data.equals(float.class)) {
                return new FloatPullConnectorImpl();
            }
            else if (_data.equals(char.class)) {
                return new CharPullConnectorImpl();
            }
            else if (_data.equals(byte.class)) {
                return new BytePullConnectorImpl();
            }
            else if (_data.equals(boolean.class)) {
                return new BoolPullConnectorImpl();
            }

            throw new ConnectionCreationException(
                "No implemented push connector for primitive type: "
                    + _data);
        }
        else {
            // first create a new generic connector
            GenericPullConnector<R, D> gpc = new GenericPullConnector<R, D>(
                _recvClass, _data);

            // System.out.println("conn class: " + _connectorClass);

            // now generate an object to wrap it in
            try {
                PullConnector pc = (PullConnector) ConnectorGenerator
                    .create(
                        _connectorClass,
                        new String[] {
                            PullConnectorOut.PULL_CONNECTOR_OUT_METHOD,
                            PullConnectorRegister.PULL_CONNECTOR_REGISTER_METHOD,
                            "stopConnector"
                        },
                        gpc,
                        new String[] {
                            PullConnectorOut.PULL_CONNECTOR_OUT_METHOD,
                            PullConnectorRegister.PULL_CONNECTOR_REGISTER_METHOD,
                            "stopConnector"
                        });
                return pc;
            }
            catch (ClassCastException e) {
                throw new ConnectionCreationException(
                    "Failure casting " + _connectorClass + " to "
                        + PullConnector.class
                        + ". Does it implement that interface?", e);
            }
        }

    }

    /**
     * @param _dataType
     * @param _recvClass
     * @param _connectorClass
     * @return
     * @throws ConnectionCreationException
     */
    private static <D, R, C> PushConnector generatePushConnector(
            Class<D> _data, Class<R> _recvClass,
            Class<C> _connectorClass)
            throws ConnectionCreationException {

        if (_data.isPrimitive()) {
            if (_data.equals(int.class)) {
                return new IntPushConnectorImpl();
            }
            else if (_data.equals(long.class)) {
                return new LongPushConnectorImpl();
            }
            else if (_data.equals(short.class)) {
                return new ShortPushConnectorImpl();
            }
            else if (_data.equals(double.class)) {
                return new DoublePushConnectorImpl();
            }
            else if (_data.equals(float.class)) {
                return new FloatPushConnectorImpl();
            }
            else if (_data.equals(char.class)) {
                return new CharPushConnectorImpl();
            }
            else if (_data.equals(byte.class)) {
                return new BytePushConnectorImpl();
            }
            else if (_data.equals(boolean.class)) {
                return new BoolPushConnectorImpl();
            }

            // really shouldn't happen!
            throw new ConnectionCreationException(
                "No implemented push connector for primitive type: "
                    + _data);
        }
        else {
            // first create a new generic connector
            GenericPushConnector<R, D> gpc = new GenericPushConnector<R, D>(
                _recvClass, _data);
            try {
                // now generate an object to wrap it in
                PushConnector pc = (PushConnector) ConnectorGenerator
                    .create(
                        _connectorClass,
                        new String[] {
                            PushConnectorOut.PUSH_CONNECTOR_OUT_METHOD,
                            PushConnectorRegister.PUSH_CONNECTOR_REGISTER_METHOD,
                            "stopConnector",
                            "flush"
                            
                        },
                        gpc,
                        new String[] {
                            PushConnectorOut.PUSH_CONNECTOR_OUT_METHOD,
                            PushConnectorRegister.PUSH_CONNECTOR_REGISTER_METHOD,
                            "stopConnector",
                            "flush"
                        });
                return pc;
            }
            catch (ClassCastException e) {
                throw new ConnectionCreationException(
                    "Failure casting " + _connectorClass + " to "
                        + PushConnector.class
                        + ". Does it implement that interface?", e);
            }

        }

    }

    /**
     * @param _pc
     * @param _connectorClass
     * @param _recv
     * @param _recvClass
     * @throws ConnectionCreationException
     */
    public static void connect(PullConnectorRegister _pc,
            Class<?> _connectorClass, PullReceiver _recv,
            Class<?> _recvClass) throws ConnectionCreationException {

        Method connectMethod = FrameworkUtils.findAssignableMethod(
            _connectorClass, _recvClass,
            PullConnectorRegister.PULL_CONNECTOR_REGISTER_METHOD, 1, 0);

        if (connectMethod != null) {
            // System.out.println("connect method:" + connectMethod);
            try {
                connectMethod.invoke(_pc, new Object[] {
                    _recv
                });
            }
            catch (IllegalArgumentException e) {
                throw new ConnectionCreationException(e);
            }
            catch (IllegalAccessException e) {
                throw new ConnectionCreationException(e);
            }
            catch (InvocationTargetException e) {
                throw new ConnectionCreationException(e);
            }
        }
        else {
            throw new ConnectionCreationException(
                "Unable to find assignable method in class: "
                    + _connectorClass
                    + " "
                    + PullConnectorRegister.PULL_CONNECTOR_REGISTER_METHOD);
        }

    }

    /**
     * @param _send
     * @param _sendClass
     * @param _pc
     * @param _connectorClass
     * @param _id
     * @throws ConnectionCreationException
     */
    public static void connect(PullSender _send, Class<?> _sendClass,
            PullConnectorOut _pc, Class<?> _connectorClass, String _id)
            throws ConnectionCreationException {
        // System.out.println(_pc.getClass());

        Method connectMethod = FrameworkUtils.findAssignableMethod(
            _sendClass, _connectorClass, PullSender.PULL_SENDER_METHOD,
            2, 1);

        if (connectMethod != null) {
            // System.out.println("connect method:" + connectMethod);
            try {
                connectMethod.invoke(_send, new Object[] {
                    _id, _pc
                });
            }
            catch (IllegalArgumentException e) {
                throw new ConnectionCreationException(e);
            }
            catch (IllegalAccessException e) {
                throw new ConnectionCreationException(e);
            }
            catch (InvocationTargetException e) {
                throw new ConnectionCreationException(e);
            }
        }
        else {
            throw new ConnectionCreationException(
                "Unable to find assignable method in class: "
                    + _sendClass + " " + PullSender.PULL_SENDER_METHOD);

        }

    }

    /**
     * @param _send
     * @param _sendClass
     * @param _pc
     * @param _connectorClass
     * @param _id
     * @throws ConnectionCreationException
     */
    public static void connect(PullSender[] _send, Class<?> _sendClass,
            PullConnectorOut _pc, Class<?> _connectorClass, String _id)
            throws ConnectionCreationException {
        // System.out.println(_pc.getClass());

        Method connectMethod = FrameworkUtils.findAssignableMethod(
            _sendClass, _connectorClass, PullSender.PULL_SENDER_METHOD,
            2, 1);

        if (connectMethod != null) {
            for (int i = 0; i < _send.length; i++) {
                // System.out.println("connect method:" +
                // connectMethod);
                try {
                    connectMethod.invoke(_send[i], new Object[] {
                        _id, _pc
                    });
                }
                catch (IllegalArgumentException e) {
                    throw new ConnectionCreationException(e);
                }
                catch (IllegalAccessException e) {
                    throw new ConnectionCreationException(e);
                }
                catch (InvocationTargetException e) {
                    throw new ConnectionCreationException(e);
                }
            }
        }
        else {
            throw new ConnectionCreationException(
                "Unable to find assignable method in class: "
                    + _sendClass + " " + PullSender.PULL_SENDER_METHOD);

        }

    }

    /**
     * Create a local pull connection between the two input processes.
     * Resulting connector object is stored in this class. \todo What to
     * do about storage?
     * 
     * @param _send
     *            The process that sends the pull query.
     * @param _recv
     *            The process that receives the pull query.
     * @return
     * @throws ConnectionCreationException
     */
    public static PullConnector connect(PullSender _send,
            PullReceiver _recv, String _dataType, String _id)
            throws ConnectionCreationException {

        // System.out.println("LocalConnectionManager.connect()");

        Class<?> dataClass = LocalConnectionFactory
            .lookupClass(_dataType);

        if (dataClass == null) {
            throw new ConnectionCreationException(
                "Unknown data type for connection: \""
                    + _dataType
                    + "\". Make sure it has been added to the LocalConnectionManager");
        }

        // Now find the correct receiver interface
        Class<?> recvClass = getReceiverClassForPull(_recv.getClass(),
            dataClass);

        // System.out.println("receiver class: " + recvClass);

        Class<?> connectorClass = LocalConnectionFactory
            .lookupPullConnector(dataClass);

        if (connectorClass == null) {
            throw new ConnectionCreationException(
                "Unable to find push connector for " + dataClass
                    + " in LocalConnectionFactory");
        }

        // now create a new connector object for the connection
        PullConnector pc = generatePullConnector(dataClass, recvClass,
            connectorClass);

        Class<?> sendClass = getSenderClassForPull(_send.getClass(),
            connectorClass);

        // System.out.println("sender class: " + sendClass);

        connect(_send, sendClass, pc, connectorClass, _id);

        connect(pc, connectorClass, _recv, recvClass);

        return pc;
    }

    /**
     * Create a local pull connection between the two input processes.
     * Resulting connector object is stored in this class. \todo What to
     * do about storage?
     * 
     * @param _send
     *            The process that sends the pull query.
     * @param _recv
     *            The process that receives the pull query.
     * @return
     * @throws ConnectionCreationException
     */
    public static PullConnector connect(PullSender _send[],
            PullReceiver _recv, String _dataType, String _id)
            throws ConnectionCreationException {

        // System.out.println("LocalConnectionManager.connect()");

        Class<?> dataClass = LocalConnectionFactory
            .lookupClass(_dataType);

        if (dataClass == null) {
            throw new ConnectionCreationException(
                "Unknown data type for connection: \""
                    + _dataType
                    + "\". Make sure it has been added to the LocalConnectionManager");
        }

        // Now find the correct receiver interface
        Class<?> recvClass = getReceiverClassForPull(_recv.getClass(),
            dataClass);

        // System.out.println("receiver class: " + recvClass);

        Class<?> connectorClass = LocalConnectionFactory
            .lookupPullConnector(dataClass);

        if (connectorClass == null) {
            throw new ConnectionCreationException(
                "Unable to find push connector for " + dataClass
                    + " in LocalConnectionFactory");
        }

        // now create a new connector object for the connection
        PullConnector pc = generatePullConnector(dataClass, recvClass,
            connectorClass);

        Class<?> sendClass = getSenderClassForPull(_send[0].getClass(),
            connectorClass);

        // System.out.println("sender class: " + sendClass);
        connect(_send, sendClass, pc, connectorClass, _id);
        connect(pc, connectorClass, _recv, recvClass);

        return pc;

    }

    /**
     * @param _register
     * @param _connectorClass
     * @param _recv
     * @param _recvClass
     * @throws ConnectionCreationException
     */
    public static void connect(PushConnectorRegister _pc,
            Class<?> _connectorClass, PushReceiver _recv,
            Class<?> _recvClass) throws ConnectionCreationException {

        Method connectMethod = FrameworkUtils.findAssignableMethod(
            _connectorClass, _recvClass,
            PushConnectorRegister.PUSH_CONNECTOR_REGISTER_METHOD, 1, 0);

        if (connectMethod != null) {
            // System.out.println("connect method:" + connectMethod);
            try {
                connectMethod.invoke(_pc, new Object[] {
                    _recv
                });
            }
            catch (IllegalArgumentException e) {
                throw new ConnectionCreationException(e);
            }
            catch (IllegalAccessException e) {
                throw new ConnectionCreationException(e);
            }
            catch (InvocationTargetException e) {
                throw new ConnectionCreationException(e);
            }
        }
        else {
            throw new ConnectionCreationException(
                "Unable to find assignable method in class: "
                    + _connectorClass
                    + " "
                    + PushConnectorRegister.PUSH_CONNECTOR_REGISTER_METHOD);
        }
    }

    /**
     * @param _send
     * @param _sendClass
     * @param _out
     * @param _connectorClass
     * @throws ConnectionCreationException
     */
    public static void connect(PushSender _send, Class<?> _sendClass,
            PushConnectorOut _pc, Class<?> _connectorClass,
            String _connectionID) throws ConnectionCreationException {

        // System.out.println(_pc.getClass());

        Method connectMethod = FrameworkUtils.findAssignableMethod(
            _sendClass, _connectorClass, PushSender.PUSH_SENDER_METHOD,
            2, 1);

        if (connectMethod != null) {
            // System.out.println("connect method:" + connectMethod);
            try {
                connectMethod.invoke(_send, new Object[] {
                    _connectionID, _pc
                });
            }
            catch (IllegalArgumentException e) {
                throw new ConnectionCreationException(e);
            }
            catch (IllegalAccessException e) {
                throw new ConnectionCreationException(e);
            }
            catch (InvocationTargetException e) {
                throw new ConnectionCreationException(e);
            }
        }
        else {
            throw new ConnectionCreationException(
                "Unable to find assignable method in class: "
                    + _sendClass + " " + PushSender.PUSH_SENDER_METHOD);

        }

    }

    /**
     * Create a local push connection between the two input processes.
     * Resulting connector object is stored in this class. \todo What to
     * do about storage?
     * 
     * @param _send
     *            The process that sends the push data.
     * @param _recv
     *            The process that receivers the push data.
     * @param _dataType
     *            TODO
     * @return
     * @throws ConnectionCreationException
     */
    public static PushConnector connect(PushSender[] _send,
            PushReceiver[] _recv, String _dataType, String _id)
            throws ConnectionCreationException {

        // this should have been checked previously!
        assert _recv.length > 0;
        // this should have been checked previously!
        assert _send.length > 0;

        // System.out.println(_id);
        // System.out.println("LocalConnectionManager.connect(): " +
        // _recv[0].getClass());

        Class<?> dataClass = LocalConnectionFactory
            .lookupClass(_dataType);

        if (dataClass == null) {
            throw new ConnectionCreationException(
                "Unknown data type for connection: \""
                    + _dataType
                    + "\". Make sure it has been added to the LocalConnectionManager");
        }

        // Now find the correct receiver interface
        Class<?> recvClass = getReceiverClassForPush(_recv[0]
            .getClass(), dataClass);

        if (recvClass == null) {
            throw new ConnectionCreationException(
                "Unable to find data type " + _recv.getClass());
        }

        // System.out.println("receiver class: " + recvClass);

        Class<?> connectorClass = LocalConnectionFactory
            .lookupPushConnector(dataClass);

        if (connectorClass == null) {
            throw new ConnectionCreationException(
                "Unable to find push connector for " + dataClass
                    + " in LocalConnectionFactory");
        }

        // now create a new connector object for the connection
        PushConnector pc = generatePushConnector(dataClass, recvClass,
            connectorClass);

        Class<?> sendClass = getSenderClassForPush(_send[0].getClass(),
            connectorClass);

        // System.out.println("sender class: " + sendClass);

        for (int i = 0; i < _send.length; i++) {
            connect(_send[i], sendClass, (PushConnectorOut) pc,
                connectorClass, _id);
        }

        for (int i = 0; i < _recv.length; i++) {
            connect((PushConnectorRegister) pc, connectorClass,
                _recv[i], recvClass);
        }

        return pc;

    }

    /**
     * Look for the interface that supports receiving data of the input
     * type.
     * 
     * @param _class
     * @param _class2
     * @return
     * @throws ConnectionCreationException
     */
    public static Class getReceiverClassForPull(
            Class<? extends PullReceiver> _recvClass, Class _dataClass)
            throws ConnectionCreationException {

        Class<?> interf = FrameworkUtils.findReturningInterfacePrefix(
            _recvClass, _dataClass, PullReceiver.PULL_RECEIVER_METHOD,
            1);

        if (interf != null) {
            return interf;
        }

        throw new ConnectionCreationException(
            "Cannot find suitable receiver interface in class. recv = "
                + _recvClass.getCanonicalName() + " data = "
                + _dataClass.getCanonicalName());
    }

    /**
     * Look for the interface that supports receiving data of the input
     * type.
     * 
     * @param _class
     * @param _class2
     * @return
     * @throws ConnectionCreationException
     */
    public static Class getReceiverClassForPush(
            Class<? extends PushReceiver> _recvClass, Class _dataClass)
            throws ConnectionCreationException {

        // System.out
        // .println("LocalConnectionManager.getReceiverClassForPush()");
        // System.out.println(_recvClass);
        // System.out.println(_dataClass);

        Class<?> interf = FrameworkUtils.findAssignmentInterface(
            _recvClass, _dataClass, PushReceiver.PUSH_RECEIVER_METHOD,
            2, 1);

        if (interf != null) {
            return interf;
        }

        throw new ConnectionCreationException(
            "Cannot find suitable receiver interface in class. recv class = "
                + _recvClass.getCanonicalName() + " data class = "
                + _dataClass.getCanonicalName());
    }

    /**
     * Discover the sender interface that
     * 
     * @param _sendClass
     * @param _connectorClass
     * @return
     * @throws ConnectionCreationException
     */
    public static Class<?> getSenderClassForPull(
            Class<? extends PullSender> _sendClass,
            Class<?> _connectorClass)
            throws ConnectionCreationException {

        // look at each interface in the sender class and find one that
        // supports... setPullConnector(String, _connectorClass)

        Class<?> interf = FrameworkUtils.findAssignmentInterface(
            _sendClass, _connectorClass, PullSender.PULL_SENDER_METHOD,
            2, 1);

        if (interf != null) {
            return interf;
        }

        throw new ConnectionCreationException(
            "Cannot find suitable receiver interface in class. send = "
                + _sendClass.getCanonicalName() + " connector = "
                + _connectorClass.getCanonicalName());

    }

    /**
     * Discover the sender interface that
     * 
     * @param _sendClass
     * @param _connectorClass
     * @return
     * @throws ConnectionCreationException
     */
    public static Class<?> getSenderClassForPush(
            Class<? extends PushSender> _sendClass,
            Class<?> _connectorClass)
            throws ConnectionCreationException {

        // look at each interface in the sender class and find one that
        // supports... setPushConnector(String, _connectorClass)

        Class<?> interf = FrameworkUtils.findAssignmentInterface(
            _sendClass, _connectorClass, PushSender.PUSH_SENDER_METHOD,
            2, 1);

        if (interf != null) {
            return interf;
        }

        throw new ConnectionCreationException(
            "Cannot find suitable receiver interface in class. send = "
                + _sendClass.getCanonicalName() + " connector = "
                + _connectorClass.getCanonicalName());

    }

}
