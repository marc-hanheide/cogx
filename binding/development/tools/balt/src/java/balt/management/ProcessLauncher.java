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

package balt.management;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Properties;

import balt.corba.autogen.FrameworkBasics.BALTTime;
import balt.corba.autogen.FrameworkBasics.FrameworkConnectionType;
import balt.corba.autogen.FrameworkBasics.ProcessConfigurationMap;
import balt.corba.autogen.FrameworkBasics.ProcessConnection;
import balt.corba.autogen.FrameworkBasics.ProcessDescription;
import balt.corba.autogen.FrameworkBasics.ProcessLanguage;
import balt.corba.impl.RemoteConnectionManager;
import balt.core.connectors.ConnectionCreationException;
import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.FrameworkConnector;
import balt.core.connectors.LocalConnectionManager;
import balt.core.connectors.pull.PullConnector;
import balt.core.connectors.pull.PullReceiver;
import balt.core.connectors.pull.PullSender;
import balt.core.connectors.push.PushConnector;
import balt.core.connectors.push.PushReceiver;
import balt.core.connectors.push.PushSender;
import balt.core.processes.FrameworkProcess;
import balt.jni.NativeProcessLauncher;

// TODO Must rationalise storage vs. static

/**
 * Class that does most of the work to decide which processes should be
 * launched and connected in which way. Could do with a couple of
 * improvements. \todo Add connector shutdown to prevent annoying exit
 * errors.
 * 
 * @author nah
 */
public class ProcessLauncher {

    private static ArrayList<CPPProcessDescription> m_cppProcesses;
    private static HashMap<JavaProcessDescription, FrameworkProcess> m_createdProcesses;
    private static HashMap<JavaProcessDescription, Thread> m_processThreads;
    private static ArrayList<FrameworkConnector> m_createdConnectors;

    static {
        m_createdProcesses =
                new HashMap<JavaProcessDescription, FrameworkProcess>();
        m_processThreads =
                new HashMap<JavaProcessDescription, Thread>();
        m_cppProcesses = new ArrayList<CPPProcessDescription>();
        m_createdConnectors = new ArrayList<FrameworkConnector>();
    }

    // private static boolean checkClassForInterface(Class<?> _class,
    // Class<?> _interface) {
    // Class<?>[] interfaces = _class.getInterfaces();
    // for (int i = 0; i < interfaces.length; i++) {
    // System.out.println("checking: " + interfaces[i]);
    // if (_interface.isAssignableFrom(interfaces[i])) {
    // return true;
    // }
    // }
    // return false;
    // }

    private static void connectRemotePullConnection(ConnectionDescription _desc)
            throws ProcessLauncherException,
            InvalidConnectionProcessException,
            FrameworkConnectionException, ConnectionCreationException {

        isValidConnectionDescription(_desc);
        connectRemotePullSenders(_desc);
        connectRemotePullReceivers(_desc);

    }

    private static void connectRemotePullReceivers(ConnectionDescription _desc)
            throws ProcessLauncherException,
            InvalidConnectionProcessException {

        // Only a single pull receiver
        ManagedProcessDescription procDescB =
                _desc.getProcessDescriptionB()[0];

        if (procDescB.isLocal()) {

            if (procDescB instanceof JavaProcessDescription) {

                // check for valid interfaces
                if (!isValidPullTarget((JavaProcessDescription) procDescB)) {
                    throw new InvalidConnectionProcessException(
                        "Not suitable target for remote pull: "
                            + procDescB);
                }

                // valid due to previous check
                PullReceiver processB =
                        (PullReceiver) getJavaProcess((JavaProcessDescription) procDescB);

                RemoteConnectionManager.connectToRemotePullConnector(
                    _desc, processB);
            }
            else if (procDescB instanceof CPPProcessDescription) {
                // System.out.println("need to connect cpp process
                // to remote server");
                NativeProcessLauncher
                    .connectRemotePullConnectionReceiver(
                        (CPPProcessDescription) procDescB,
                        RemoteConnectionManager
                            .generateServerName(_desc), _desc
                            .getDataType(), _desc.getId());
            }
            else {
                // should never happen due to previous checks!

                throw new ProcessLauncherException(
                    "Unknown language type!");
            }

        }

    }

    /**
     * @param _desc
     * @param remoteID
     * @throws FrameworkConnectionException
     * @throws ConnectionCreationException
     * @throws ProcessLauncherException
     */
    private static void connectRemotePullSenders(ConnectionDescription _desc)
            throws FrameworkConnectionException,
            ConnectionCreationException, ProcessLauncherException {
        ManagedProcessDescription[] procs =
                _desc.getProcessDescriptionA();

        // now iterate through the A procs and connect them to the
        // server

        String remoteID =
                RemoteConnectionManager.generateServerName(_desc);

        for (int i = 0; i < procs.length; i++) {

            // only connect local processes!
            if (procs[i].isLocal()) {
                if (procs[i] instanceof JavaProcessDescription) {
                    JavaProcessDescription jpd =
                            (JavaProcessDescription) procs[i];
                    if (isValidPullSource(jpd)) {
                        PullSender ps =
                                (PullSender) getJavaProcess(jpd);
                        RemoteConnectionManager
                            .connectToRemotePullConnector(_desc, ps);
                    }
                    else {
                        throw new ProcessLauncherException(
                            "Incorrect source for pull: "
                                + jpd.toString());
                    }
                }
                else if (procs[i] instanceof CPPProcessDescription) {
                    NativeProcessLauncher
                        .connectRemotePullConnectionSender(
                            (CPPProcessDescription) procs[i], remoteID,
                            _desc.getDataType(), _desc.getId());
                }
                else {
                    // should never happen due to previous checks!
                    throw new ProcessLauncherException(
                        "Unknown language type! " + procs[i].getClass());
                }
            }
        }
    }

    private static void connectRemotePushConnection(ConnectionDescription _desc)
            throws InvalidConnectionProcessException,
            ProcessLauncherException, FrameworkConnectionException,
            ConnectionCreationException {

        isValidConnectionDescription(_desc);
        connectRemotePushSenders(_desc);
        connectRemotePushReceivers(_desc);
    }

    private static void connectRemotePushReceivers(ConnectionDescription _desc)
            throws InvalidConnectionProcessException,
            ProcessLauncherException {

        // System.out.println("ProcessLauncher.connectRemotePushConnection()");
        // System.out.println("for: " + _desc);

        // The procA[0] ALWAYS creates the server

        ManagedProcessDescription[] procBs =
                _desc.getProcessDescriptionB();

        String remoteID =
                RemoteConnectionManager.generateServerName(_desc);

        for (int i = 0; i < procBs.length; i++) {

            ManagedProcessDescription procDescB = procBs[i];

            if (procDescB.isLocal()) {

                if (_desc.isPushConnection()) {

                    if (procDescB instanceof JavaProcessDescription) {

                        // check for valid interfaces
                        if (!isValidPushTarget((JavaProcessDescription) procDescB)) {
                            throw new InvalidConnectionProcessException(
                                "Not suitable target for remote push: "
                                    + procDescB);
                        }

                        // valid due to previous check
                        PushReceiver processB =
                                (PushReceiver) getJavaProcess((JavaProcessDescription) procDescB);

                        RemoteConnectionManager
                            .connectToRemotePushConnector(_desc,
                                processB);
                    }
                    else if (procDescB instanceof CPPProcessDescription) {
                        // System.out.println("need to connect cpp
                        // process
                        // to remote server");
                        NativeProcessLauncher
                            .connectRemotePushConnectionReceiver(
                                (CPPProcessDescription) procDescB,
                                remoteID, _desc.getDataType(), _desc
                                    .getId());
                    }
                    else {
                        // should never happen due to previous checks!
                        throw new ProcessLauncherException(
                            "Unknown language type!");
                    }

                }

            }
            else {
                // System.out.println("Was made previously");
            }
        }
    }

    /**
     * @param _desc
     * @param remoteID
     * @throws FrameworkConnectionException
     * @throws ConnectionCreationException
     * @throws ProcessLauncherException
     */
    private static void connectRemotePushSenders(ConnectionDescription _desc)
            throws FrameworkConnectionException,
            ConnectionCreationException, ProcessLauncherException {
        // now iterate through the A procs and connect them to the
        // server

        ManagedProcessDescription[] procs =
                _desc.getProcessDescriptionA();

        String remoteID =
                RemoteConnectionManager.generateServerName(_desc);

        for (int i = 0; i < procs.length; i++) {

            // only connect local processes!
            if (procs[i].isLocal()) {
                if (procs[i] instanceof JavaProcessDescription) {
                    JavaProcessDescription jpd =
                            (JavaProcessDescription) procs[i];
                    if (isValidPushSource(jpd)) {
                        PushSender ps =
                                (PushSender) getJavaProcess(jpd);
                        RemoteConnectionManager
                            .connectToRemotePushConnector(_desc, ps);
                    }
                    else {
                        throw new ProcessLauncherException(
                            "Incorrect source for push: "
                                + jpd.toString());
                    }
                }
                else if (procs[i] instanceof CPPProcessDescription) {
                    NativeProcessLauncher
                        .connectRemotePushConnectionSender(
                            (CPPProcessDescription) procs[i], remoteID,
                            _desc.getDataType(), _desc.getId());

                }
                else {
                    // should never happen due to previous checks!
                    throw new ProcessLauncherException(
                        "Unknown language type! " + procs[i].getClass());
                }
            }
        }
    }

    private static void createLocalCPPPullConnection(CPPProcessDescription[] processDescriptionA,
                                                     CPPProcessDescription processDescriptionB,
                                                     String _dataType,
                                                     String _id) {
        // Native code does the checking
        NativeProcessLauncher.createLocalPullConnection(
            processDescriptionA, processDescriptionB, _dataType, _id);
    }

    private static void createLocalCPPPushConnection(CPPProcessDescription[] processDescriptionA,
                                                     CPPProcessDescription[] processDescriptionB,
                                                     String _dataType,
                                                     String _id) {
        // Native code does the checking
        NativeProcessLauncher.createLocalPushConnection(
            processDescriptionA, processDescriptionB, _dataType, _id);
    }

    private static void createLocalJavaPullConnection(JavaProcessDescription[] processDescriptionA,
                                                      JavaProcessDescription processDescriptionB,
                                                      String _dataType,
                                                      String _id)
            throws InvalidConnectionProcessException,
            ConnectionCreationException {

        PullSender[] processA =
                getJavaPullSenderProcesses(processDescriptionA);

        if (!isValidPullTarget(processDescriptionB)) {
            throw new InvalidConnectionProcessException(
                "Not suitable target for local pull: "
                    + processDescriptionA);
        }
        PullReceiver processB =
                (PullReceiver) getJavaProcess(processDescriptionB);

        PullConnector pc =
                LocalConnectionManager.connect(processA, processB,
                    _dataType, _id);

        m_createdConnectors.add(pc);
    }

    private static void createLocalJavaPushConnection(JavaProcessDescription[] processDescriptionA,
                                                      JavaProcessDescription[] processDescriptionB,
                                                      String _dataType,
                                                      String _id)
            throws InvalidConnectionProcessException,
            ConnectionCreationException {

        // System.out
        // .println("ProcessLauncher.createLocalJavaPushConnection()");

        // check for valid interfaces
        for (int i = 0; i < processDescriptionA.length; i++) {
            // System.out.println(processDescriptionA[i]);
            if (!isValidPushSource(processDescriptionA[i])) {
                throw new InvalidConnectionProcessException(
                    "Not suitable source for local push: "
                        + processDescriptionA[i]);
            }
        }

        for (int i = 0; i < processDescriptionB.length; i++) {
            // System.out.println(processDescriptionB[i]);
            if (!isValidPushTarget(processDescriptionB[i])) {
                throw new InvalidConnectionProcessException(
                    "Not suitable target for local push: "
                        + processDescriptionB[i]);
            }
        }

        // valid due to previous check
        PushSender[] processA =
                getJavaPushSenderProcesses(processDescriptionA);
        PushReceiver[] processB =
                getJavaPushReceiverProcesses(processDescriptionB);

        PushConnector pc =
                LocalConnectionManager.connect(processA, processB,
                    _dataType, _id);
        m_createdConnectors.add(pc);
    }

    /**
     * @param procDesc
     * @throws ProcessLauncherException
     */
    private static void createLocalProcess(ManagedProcessDescription procDesc)
            throws ProcessLauncherException {
        if (procDesc instanceof JavaProcessDescription) {
            // System.out.println("ProcessLauncher.createLocalProcess()");
            // System.out.println(procDesc.getProcessConfiguration());
            getJavaProcess((JavaProcessDescription) procDesc);
        }
        else if (procDesc instanceof CPPProcessDescription) {
            if (!m_cppProcesses.contains(procDesc)) {
                NativeProcessLauncher
                    .launchProcess((CPPProcessDescription) procDesc);
                // store for later to prevent multiple launches
                m_cppProcesses.add((CPPProcessDescription) procDesc);
            }
        }
        else {
            // should never happen due to previous checks!
            throw new ProcessLauncherException("Unknown language type!");
        }
    }

    /**
     * Create a push connection between 2 processes created locally
     * 
     * @param _desc
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     * @throws ConnectionCreationException
     */
    private static void createLocalPullConnection(ConnectionDescription _desc)
            throws InvalidConnectionProcessException,
            ProcessLauncherException, ConnectionCreationException {

        // System.out.println("ProcessLauncher.createLocalPushConnection()");

        // sanity check, probably redundant
        isValidConnectionDescription(_desc);

        // pulls can be sent by many, but received only by one

        ManagedProcessDescription[] processDescriptionA =
                _desc.getProcessDescriptionA();
        ManagedProcessDescription processDescriptionB =
                _desc.getProcessDescriptionB()[0];

        // only check one, because previously we've determined they are
        // the same language
        if (processDescriptionB instanceof JavaProcessDescription) {
            JavaProcessDescription[] javaProcA =
                    toJavaProcessDescriptions(processDescriptionA);
            JavaProcessDescription javaProcB =
                    (JavaProcessDescription) processDescriptionB;
            createLocalJavaPullConnection(javaProcA, javaProcB, _desc
                .getDataType(), _desc.getId());
        }
        else if (processDescriptionB instanceof CPPProcessDescription) {
            CPPProcessDescription[] cppProcA =
                    toCPPProcessDescriptions(processDescriptionA);
            CPPProcessDescription cppProcB =
                    (CPPProcessDescription) processDescriptionB;
            createLocalCPPPullConnection(cppProcA, cppProcB, _desc
                .getDataType(), _desc.getId());
        }
        else {
            // should never happen due to previous checks!
            throw new ProcessLauncherException("Unknown language type!");
        }

    }

    /**
     * Create a push connection between 2 processes created locally
     * 
     * @param _desc
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     * @throws ConnectionCreationException
     */
    private static void createLocalPushConnection(ConnectionDescription _desc)
            throws InvalidConnectionProcessException,
            ProcessLauncherException, ConnectionCreationException {

        // updated for arrays

        // System.out.println("ProcessLauncher.createLocalPushConnection()");

        ManagedProcessDescription[] processDescriptionA =
                _desc.getProcessDescriptionA();
        ManagedProcessDescription[] processDescriptionB =
                _desc.getProcessDescriptionB();

        // only check one, because previously we've determined they are
        // the same language
        if (processDescriptionA[0] instanceof JavaProcessDescription) {
            JavaProcessDescription[] javaProcA =
                    toJavaProcessDescriptions(processDescriptionA);
            JavaProcessDescription[] javaProcB =
                    toJavaProcessDescriptions(processDescriptionB);
            createLocalJavaPushConnection(javaProcA, javaProcB, _desc
                .getDataType(), _desc.getId());
        }
        else if (processDescriptionA[0] instanceof CPPProcessDescription) {
            CPPProcessDescription[] cppProcA =
                    toCPPProcessDescriptions(processDescriptionA);
            CPPProcessDescription[] cppProcB =
                    toCPPProcessDescriptions(processDescriptionB);
            createLocalCPPPushConnection(cppProcA, cppProcB, _desc
                .getDataType(), _desc.getId());
        }
        else {
            // should never happen due to previous checks!
            throw new ProcessLauncherException("Unknown language type!");
        }

    }

    @SuppressWarnings("unchecked")
    static private FrameworkProcess createNewProcess(JavaProcessDescription _desc) {

        try {

            // System.out.println("ProcessLauncher.createNewProcess()");
            // System.out.println("desc: " +
            // _desc.getProcessConfiguration());
            // use thse string contructur to set name
            Constructor<FrameworkProcess> frameworkConstructor =
                    _desc.getProcessClass()
                        .getConstructor(String.class);

            // create the instance
            FrameworkProcess process =
                    frameworkConstructor.newInstance(_desc
                        .getProcessName());

            process.configure(_desc.getProcessConfiguration());

            // store process
            m_createdProcesses.put(_desc, process);

            return process;
        }
        catch (SecurityException e) {
            e.printStackTrace();
            System.exit(1);
        }
        catch (NoSuchMethodException e) {
            e.printStackTrace();
            System.exit(1);
        }
        catch (IllegalArgumentException e) {
            e.printStackTrace();
            System.exit(1);
        }
        catch (InstantiationException e) {
            e.printStackTrace();
            System.exit(1);
        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
            System.exit(1);
        }
        catch (InvocationTargetException e) {
            e.printStackTrace();
            System.exit(1);
        }
        return null;
    }

    /**
     * @param _desc // *
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     * @throws ConnectionCreationException
     */
    private static void createRemoteConnection(ConnectionDescription _desc)
            throws InvalidConnectionProcessException,
            ProcessLauncherException, ConnectionCreationException {

        // updated to arrays

//        System.out.println("ProcessLauncher.createRemoteConnection()");
//        System.out.println("for: " + _desc);

        // sanity checks
        isValidConnectionDescription(_desc);

        // The procA[0] ALWAYS creates the server regardless of language
        ManagedProcessDescription procDescA =
                (_desc.getProcessDescriptionA())[0];

        if (!procDescA.isLocal()) {
            // servers will be created elsewhere
            return;
        }

        if (_desc.isPushConnection()) {
            createRemotePushConnection(_desc);
        }
        else if (_desc.isPullConnection()) {
            createRemotePullConnection(_desc);
        }

    }

    /**
     * @param _desc
     * @param cppProcDesc
     * @throws ProcessLauncherException
     */
    private static void createRemoteCPPPullConnection(ConnectionDescription _desc,
                                                      CPPProcessDescription cppProcDesc)
            throws ProcessLauncherException {
        try {

            String remoteID =
                    RemoteConnectionManager
                        .createRemotePullConnectionServer(_desc);

            // System.out.println("created but not
            // connected");
            NativeProcessLauncher.connectRemotePullConnectionSender(
                cppProcDesc, remoteID, _desc.getDataType(), _desc
                    .getId());
            // System.out.println("created and sender
            // connected");
        }
        catch (FrameworkConnectionException e) {
            throw new ProcessLauncherException("", e);
        }
    }

    /**
     * @param _desc
     * @param javaProcDesc
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     */
    private static void createRemoteJavaPullConnection(ConnectionDescription _desc,
                                                       JavaProcessDescription javaProcDesc)
            throws InvalidConnectionProcessException,
            ProcessLauncherException {
        // check for valid interfaces
        if (!isValidPullSource(javaProcDesc)) {
            throw new InvalidConnectionProcessException(
                "Not suitable source for local pull: " + javaProcDesc);
        }

        // valid due to previous check
        PullSender processA = (PullSender) getJavaProcess(javaProcDesc);
        try {
            RemoteConnectionManager.createRemotePullConnectionServer(
                _desc, processA);
        }
        catch (FrameworkConnectionException e) {
            throw new ProcessLauncherException("", e);
        }
    }

    /**
     * @param _desc
     * @param procDescA
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     * @throws ConnectionCreationException
     */
    private static void createRemotePullConnection(ConnectionDescription _desc)
            throws InvalidConnectionProcessException,
            ProcessLauncherException, ConnectionCreationException {

        try {

            // first, create the connection server
            RemoteConnectionManager
                .createRemotePullConnectionServer(_desc);

            // System.out.println("created but not
            // connected");
        }
        catch (FrameworkConnectionException e) {
            throw new ProcessLauncherException(e);
        }

    }

    /**
     * @param _desc
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     * @throws ConnectionCreationException
     */
    private static void createRemotePushConnection(ConnectionDescription _desc)
            throws InvalidConnectionProcessException,
            ProcessLauncherException, ConnectionCreationException {

        try {

            // first, create the connection server
            RemoteConnectionManager
                .createRemotePushConnectionServer(_desc);

            // System.out.println("created but not
            // connected");

        }
        catch (FrameworkConnectionException e) {
            throw new ProcessLauncherException(e);
        }

    }

    private static FrameworkProcess getJavaProcess(JavaProcessDescription processDescriptionA) {

        if (m_createdProcesses.containsKey(processDescriptionA)) {
            return m_createdProcesses.get(processDescriptionA);
        }
        else {
            return createNewProcess(processDescriptionA);
        }
    }

    // no longer used!
    // /**
    // * @param _desc
    // * @param cppProcDesc
    // * @throws ProcessLauncherException
    // */
    // private static void createRemoteCPPPushConnection(
    // ConnectionDescription _desc,
    // CPPProcessDescription cppProcDesc)
    // throws ProcessLauncherException {
    // try {
    // String remoteID = RemoteConnectionManager
    // .createRemotePushConnectionServer(_desc);
    // // System.out.println("created but not
    // // connected");
    // NativeProcessLauncher.connectRemotePushConnectionSender(
    // cppProcDesc, remoteID, _desc.getDataType(), _desc
    // .getId());
    // // System.out.println("created and sender
    // // connected");
    // }
    // catch (FrameworkConnectionException e) {
    // throw new ProcessLauncherException(e);
    // }
    // }

    private static PullReceiver[] getJavaPullReceiverProcesses(JavaProcessDescription[] _descriptions) {
        PullReceiver[] senders = new PullReceiver[_descriptions.length];
        for (int i = 0; i < _descriptions.length; i++) {
            senders[i] =
                    (PullReceiver) getJavaProcess(_descriptions[i]);
        }
        return senders;
    }

    // no longer used
    // /**
    // * @param _desc
    // * @param javaProcDesc
    // * @throws InvalidConnectionProcessException
    // * @throws ProcessLauncherException
    // */
    // private static void createRemoteJavaPushConnection(
    // ConnectionDescription _desc,
    // JavaProcessDescription javaProcDesc)
    // throws InvalidConnectionProcessException,
    // ProcessLauncherException {
    // // check for valid interfaces
    // if (!isValidPushSource(javaProcDesc)) {
    // throw new InvalidConnectionProcessException(
    // "Not suitable source for local push: " + javaProcDesc);
    // }
    //
    // // valid due to previous check
    // StringPushInterface.StringPushSender processA =
    // (StringPushInterface.StringPushSender)
    // getJavaProcess(javaProcDesc);
    // try {
    // RemoteConnectionManager.createRemotePushConnectionServer(
    // _desc, processA);
    // }
    // catch (FrameworkConnectionException e) {
    // throw new ProcessLauncherException("", e);
    // }
    // }

    private static PullSender[] getJavaPullSenderProcesses(JavaProcessDescription[] _descriptions) {
        PullSender[] senders = new PullSender[_descriptions.length];
        for (int i = 0; i < _descriptions.length; i++) {
            senders[i] = (PullSender) getJavaProcess(_descriptions[i]);
        }
        return senders;
    }

    private static PushReceiver[] getJavaPushReceiverProcesses(JavaProcessDescription[] _descriptions) {
        PushReceiver[] senders = new PushReceiver[_descriptions.length];
        for (int i = 0; i < _descriptions.length; i++) {
            senders[i] =
                    (PushReceiver) getJavaProcess(_descriptions[i]);
        }
        return senders;
    }

    private static PushSender[] getJavaPushSenderProcesses(JavaProcessDescription[] _descriptions) {
        PushSender[] senders = new PushSender[_descriptions.length];
        for (int i = 0; i < _descriptions.length; i++) {
            senders[i] = (PushSender) getJavaProcess(_descriptions[i]);
        }
        return senders;
    }

    /**
     * Checks whether the input describes a valid connection. Currently
     * only checks the count of processes at either end of the
     * connection.
     * 
     * @param _desc
     * @throws ProcessLauncherException
     */
    private static void isValidConnectionDescription(ConnectionDescription _desc)
            throws ProcessLauncherException {
        if (_desc.getConnectionType() == ConnectionDescription.PUSH_TO) {
            if (_desc.getProcessDescriptionA().length <= 0) {
                throw new ProcessLauncherException(
                    "More than 0 sender processes needed for connection " + _desc.getId() + " using type " + _desc.getDataType());
            }
            if (_desc.getProcessDescriptionB().length <= 0) {
                throw new ProcessLauncherException(
                    "More than 0 receiver processes needed for connection " + _desc.getId() + " using type " + _desc.getDataType());
            }
        }
        else if (_desc.getConnectionType() == ConnectionDescription.PULL_FROM) {
            if (_desc.getProcessDescriptionA().length <= 0) {
                throw new ProcessLauncherException(
                    "More than 0 sender processes needed for connection " + _desc.getId() + " using type " + _desc.getDataType());
            }
            if (_desc.getProcessDescriptionB().length != 1) {
                throw new ProcessLauncherException(
                    "Exactly 1 receiver process needed for connection " + _desc.getId() + " using type " + _desc.getDataType());
            }
        }
        else {
            throw new ProcessLauncherException(
                "Unknown connection type");
        }
    }

    @SuppressWarnings("unchecked")
    private static boolean isValidPullSource(JavaProcessDescription _proc) {
        return PullSender.class.isAssignableFrom(_proc
            .getProcessClass());
    }

    @SuppressWarnings("unchecked")
    private static boolean isValidPullTarget(JavaProcessDescription _proc) {
        return PullReceiver.class.isAssignableFrom(_proc
            .getProcessClass());
    }

    @SuppressWarnings("unchecked")
    private static boolean isValidPushSource(JavaProcessDescription _proc) {
        return PushSender.class.isAssignableFrom(_proc
            .getProcessClass());
    }

    @SuppressWarnings("unchecked")
    private static boolean isValidPushTarget(JavaProcessDescription _proc) {
        return PushReceiver.class.isAssignableFrom(_proc
            .getProcessClass());
    }

    /**
     * @param _processDescriptionA
     * @return
     */
    private static CPPProcessDescription[] toCPPProcessDescriptions(ManagedProcessDescription[] _processDescriptionA) {
        CPPProcessDescription[] cpds =
                new CPPProcessDescription[_processDescriptionA.length];
        for (int i = 0; i < _processDescriptionA.length; i++) {
            cpds[i] = (CPPProcessDescription) _processDescriptionA[i];
        }
        return cpds;
    }

    /**
     * @param _processDescriptionA
     * @return
     */
    private static JavaProcessDescription[] toJavaProcessDescriptions(ManagedProcessDescription[] _processDescriptionA) {
        JavaProcessDescription[] jpds =
                new JavaProcessDescription[_processDescriptionA.length];
        for (int i = 0; i < _processDescriptionA.length; i++) {
            jpds[i] = (JavaProcessDescription) _processDescriptionA[i];
        }
        return jpds;
    }

    /**
     * Connect local and remote processes as appropriate.
     * 
     * @param _graph
     *            The connection descriptions.
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     * @throws ConnectionCreationException
     * @throws FrameworkConnectionException
     */
    public static void connectGraph(ConnectionGraph _graph)
            throws InvalidConnectionProcessException,
            ProcessLauncherException, ConnectionCreationException,
            FrameworkConnectionException {

        ConnectionDescription desc;

        // Connect up processes
        for (Iterator<ConnectionDescription> i = _graph.iterator(); i
            .hasNext();) {
            desc = i.next();

            // System.out.println(desc);

            if (desc.isPushConnection()) {
                if (desc.isLocalConnection()
                    && desc.isSameLanguageConnection()) {
                    createLocalPushConnection(desc);
                }
                else {
                    connectRemotePushConnection(desc);
                }
            }
            else if (desc.isPullConnection()) {
                if (desc.isLocalConnection()
                    && desc.isSameLanguageConnection()) {
                    createLocalPullConnection(desc);
                }
                else {
                    connectRemotePullConnection(desc);
                }
            }
        }
    }

    /**
     * Create the processes for the connections.
     * 
     * @param _graph
     *            The connection descriptions.
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     */
    public static void createGraphProcesses(ConnectionGraph _graph)
            throws InvalidConnectionProcessException,
            ProcessLauncherException {

        ConnectionDescription desc;
        ManagedProcessDescription procDesc;
        ManagedProcessDescription[] procDescArray;

        // Connect up processes
        for (Iterator<ConnectionDescription> i = _graph.iterator(); i
            .hasNext();) {
            desc = i.next();

            // do some sanity checking
            isValidConnectionDescription(desc);

            procDescArray = desc.getProcessDescriptionA();
            // System.out.println(desc);
            for (int j = 0; j < procDescArray.length; j++) {
                // create the 2 processes if they're local
                procDesc = procDescArray[j];
                if (procDesc.isLocal()) {
                    createLocalProcess(procDesc);
                }
            }

            procDescArray = desc.getProcessDescriptionB();
            // System.out.println(desc);
            for (int j = 0; j < procDescArray.length; j++) {
                // create the 2 processes if they're local
                procDesc = procDescArray[j];
                if (procDesc.isLocal()) {
                    createLocalProcess(procDesc);
                }
            }
        }
    }

    /**
     * Create remote connection objects as necessary. Remote connections
     * are currently created for cross-language and cross network
     * processes.
     * 
     * @param _graph
     *            The connection descriptions.
     * @throws InvalidConnectionProcessException
     * @throws ProcessLauncherException
     * @throws ConnectionCreationException
     */
    public static void createRemoteConnections(ConnectionGraph _graph)
            throws InvalidConnectionProcessException,
            ProcessLauncherException, ConnectionCreationException {

        ConnectionDescription desc;

        // Create remote connection objects
        for (Iterator<ConnectionDescription> i = _graph.iterator(); i
            .hasNext();) {
            desc = i.next();

            // System.out.println(desc);

            // we do this regardless of language
            if (!(desc.isLocalConnection() && desc
                .isSameLanguageConnection())) {
                createRemoteConnection(desc);
            }

        }
    }

    /**
     * Run all created process.
     * 
     * @param _graph
     *            The connection descriptions.
     */
    public static void startGraph(ConnectionGraph _graph) {
   	 // Start all Java processes
        Iterator<JavaProcessDescription> keys =
                m_createdProcesses.keySet().iterator();

        JavaProcessDescription key;

        FrameworkProcess proc;

        // first call start on each process

        while (keys.hasNext()) {
            key = keys.next();
            // System.out.println("starting: " + key.getProcessName());
            proc = m_createdProcesses.get(key);
            proc.start();
        }

        // start all CPP processes
        for (Iterator i = m_cppProcesses.iterator(); i.hasNext();) {
            CPPProcessDescription desc =
                    (CPPProcessDescription) i.next();
            NativeProcessLauncher.startProcess(desc);
        }

        // give everything a little time to settle... this is because in
        // caat systems the start methods trigger threads that may go
        // beyond the end of the start method
        try {
            Thread.sleep(1000);
        }
        catch (InterruptedException e) {
            e.printStackTrace();
        }
       
    }

    // /**
    // * Run all created process.
    // *
    // * @param _graph
    // * The connection descriptions.
    // */
    // public static void startGraph(ConnectionGraph _graph) {
    //
    //        
    //        
    // // Start all Java processes
    // Iterator<JavaProcessDescription> keys = m_createdProcesses
    // .keySet().iterator();
    // JavaProcessDescription key;
    // FrameworkProcess proc;
    // Thread procThread;
    // while (keys.hasNext()) {
    // key = keys.next();
    // // System.out.println("starting: " + key.getProcessName());
    // proc = m_createdProcesses.get(key);
    // procThread = new Thread(proc, key.getProcessName()
    // + " Thread");
    //
    // proc.start();
    // procThread.start();
    //
    // m_processThreads.put(key, procThread);
    // }
    //
    // // start all CPP processes
    // for (Iterator i = m_cppProcesses.iterator(); i.hasNext();) {
    // CPPProcessDescription desc = (CPPProcessDescription) i
    // .next();
    // NativeProcessLauncher.startProcess(desc);
    // }
    //        
    //        
    // }

    /**
     * Stop all created processes.
     * 
     * @param _graph
     *            The connection descriptions.
     */
    public static void stopGraph(ConnectionGraph _graph) {
        // Stop all Java processes

        // Start all Java processes
        Iterator<JavaProcessDescription> keys =
                m_createdProcesses.keySet().iterator();
        JavaProcessDescription key;
        FrameworkProcess proc;
        Thread procThread;
        while (keys.hasNext()) {

            key = keys.next();

             System.out.println("stopping: " + key.getProcessName());
            proc = m_createdProcesses.get(key);

            // tell the process to stop processing
            proc.stop();

            procThread = m_processThreads.get(key);
            // wait for it to finish

            // m_createdProcesses.remove(key);
            // m_processThreads.remove(key);

            try {
                 System.out.print("waiting for " +
                 key.getProcessName()
                 + " thread... ");
                procThread.join();
                 System.out.println("OK");
            }
            catch (InterruptedException e) {
                System.out.println("ERROR JOINING THREAD!");
                e.printStackTrace();
            }
        }

        m_createdProcesses.clear();
        m_processThreads.clear();

        // start all CPP processes
        for (Iterator i = m_cppProcesses.iterator(); i.hasNext();) {
            CPPProcessDescription desc =
                    (CPPProcessDescription) i.next();
             System.out.print("calling time on: "
             + desc.getProcessName());
            NativeProcessLauncher.stopProcess(desc);
             System.out.println(".. OK");
        }

        // System.out.println("cleaning up interface");
        NativeProcessLauncher.cleanupInterface();
        m_cppProcesses.clear();

        // stop connectopns
        for (FrameworkConnector connector : m_createdConnectors) {
            connector.stopConnector();
        }

        RemoteConnectionManager.cleanup();
    }

    /**
     * Convert a CORBA ProcessConnection struct into a more managable
     * ConnectionDescription object.
     * 
     * @param _connection
     * @return
     * @throws UnknownHostException
     * @throws ClassNotFoundException
     * @throws UnknownConnectionTypeException
     * @throws ProcessLauncherException
     */
    public static ConnectionDescription toConnectionDescription(ProcessConnection _connection)
            throws UnknownHostException, ClassNotFoundException,
            UnknownConnectionTypeException, ProcessLauncherException {

        int iConnectionType;

        if (_connection.m_connectionType == FrameworkConnectionType.PUSH_CONNECTION) {
            iConnectionType = ConnectionDescription.PUSH_TO;
        }
        else if (_connection.m_connectionType == FrameworkConnectionType.PULL_CONNECTION) {
            iConnectionType = ConnectionDescription.PULL_FROM;
        }
        else {
            throw new UnknownConnectionTypeException(
                "unknown connection type: "
                    + _connection.m_connectionType);
        }

        ProcessDescription[] descs = _connection.m_senders;
        ManagedProcessDescription[] senderProcs =
                new ManagedProcessDescription[descs.length];
        for (int i = 0; i < descs.length; i++) {
            try {
                senderProcs[i] = toManagedProcessDescription(descs[i]);
            }
            catch (ClassNotFoundException e) {
                throw new ProcessLauncherException(
                    "Unable to find java class: "
                        + descs[i].m_className);
            }

        }

        descs = _connection.m_receivers;
        ManagedProcessDescription[] receiverProcs =
                new ManagedProcessDescription[descs.length];
        for (int i = 0; i < descs.length; i++) {
            try {
                receiverProcs[i] =
                        toManagedProcessDescription(descs[i]);
            }
            catch (ClassNotFoundException e) {
                throw new ProcessLauncherException(
                    "Unable to find java class: "
                        + descs[i].m_className);
            }
        }

        return new ConnectionDescription(senderProcs, iConnectionType,
            receiverProcs, _connection.m_dataType,
            _connection.m_connectionID);
    }

    public static Properties toPropertyMap(ProcessConfigurationMap _config)
            throws ProcessLauncherException {
        String[] keys = _config.m_keys;
        String[] values = _config.m_values;

        if (keys.length != values.length) {
            throw new ProcessLauncherException(
                "configuration key/value lists of unequal length");
        }

        // System.out.println("keys length: " + keys.length);

        Properties props = new Properties();

        for (int i = 0; i < keys.length; i++) {
            // System.out.println("setting property: " + keys[i]);
            // System.out.println("setting property: " + values[i]);

            props.setProperty(keys[i], values[i]);
        }

        return props;

    }

    /**
     * Convert a CORBA ProcessDescription struct into a more managable
     * ManagedProcessDescription object. This method also determines the
     * locality of the process by comparing the given hostname with the
     * hostname of the current machine using an IP lookup.
     * 
     * @param _desc
     * @return
     * @throws ClassNotFoundException
     * @throws UnknownHostException
     * @throws ProcessLauncherException
     */
    public static ManagedProcessDescription toManagedProcessDescription(ProcessDescription _desc)
            throws ClassNotFoundException, UnknownHostException,
            ProcessLauncherException {

        if (_desc.m_language == ProcessLanguage.JAVA_PROCESS) {

            // InetAddress procHost = InetAddress
            // .getByName(_desc.m_hostName);
            // System.out.println(procHost);
            // System.out.println("local?: " +
            // procHost.equals(m_localhost));
            // boolean isLocal = procHost.equals(FrameworkUtils
            // .getLocalhost());

            boolean isLocal =
                    FrameworkUtils.areSameHost(_desc.m_hostName,
                        FrameworkUtils.getHostname());

            Class procClass = null;
            if (isLocal) {
                procClass = Class.forName(_desc.m_className);
                // System.out.println("procClass: " + procClass);
            }

            return new JavaProcessDescription(_desc.m_processName,
                procClass, isLocal,
                toPropertyMap(_desc.m_configuration));
        }
        else if (_desc.m_language == ProcessLanguage.CPP_PROCESS) {

            // InetAddress procHost = InetAddress
            // .getByName(_desc.m_hostName);
            // boolean isLocal = procHost.equals(FrameworkUtils
            // .getLocalhost());

            boolean isLocal =
                    FrameworkUtils.areSameHost(_desc.m_hostName,
                        FrameworkUtils.getHostname());

            return new CPPProcessDescription(_desc.m_processName,
                _desc.m_className, isLocal,
                toPropertyMap(_desc.m_configuration));
        }

        throw new ProcessLauncherException("Unknown language type!");
    }

    /**
     * 
     */
    public static final void resetClock() {
        // System.out.println("ProcessLauncher.resetClock()");
        NativeProcessLauncher.resetClock();
    }

    public static BALTTime getBALTTime() {
        return NativeProcessLauncher.getBALTTime();
    }

	public static void runGraph(ConnectionGraph _graph) {
	

        // then call run on each
        Thread procThread;
        Iterator<JavaProcessDescription> keys = m_createdProcesses.keySet().iterator();

        while (keys.hasNext()) {
            JavaProcessDescription key = keys.next();
            // System.out.println("starting: " + key.getProcessName());
            FrameworkProcess proc = m_createdProcesses.get(key);
            procThread =
                    new Thread(proc, key.getProcessName() + " Thread");
            procThread.start();
            m_processThreads.put(key, procThread);
        }

        // start all CPP processes
        for (Iterator i = m_cppProcesses.iterator(); i.hasNext();) {
            CPPProcessDescription desc =
                    (CPPProcessDescription) i.next();
            NativeProcessLauncher.runProcess(desc);
        }
	}

    // public static ConnectionDescription toConnectionDescriptionL2R(
    // ProcessConnection _connection) throws UnknownHostException,
    // ClassNotFoundException, UnknownConnectionTypeException {
    //
    // int iConnectionType;
    //
    // if (_connection.m_connectionType ==
    // FrameworkConnectionType.PUSH_CONNECTION) {
    // iConnectionType = ConnectionDescription.PUSH_TO;
    // }
    // else if (_connection.m_connectionType ==
    // FrameworkConnectionType.PUSH_CONNECTION) {
    // iConnectionType = ConnectionDescription.PULL_FROM;
    // }
    // else {
    // throw new UnknownConnectionTypeException("unknown connection
    // type: "
    // + _connection.m_connectionType);
    // }
    //
    // JavaProcessDescription senderProc =
    // toManagedProcessDescription(_connection.m_sender);
    // senderProc.setIsLocal(true);
    // JavaProcessDescription receiverProc =
    // toManagedProcessDescription(_connection.m_receiver);
    // receiverProc.setIsLocal(false);
    // return new ConnectionDescription(senderProc, iConnectionType,
    // receiverProc);
    // }
    //
    // public static ConnectionDescription toConnectionDescriptionR2L(
    // ProcessConnection _connection) throws UnknownHostException,
    // ClassNotFoundException, UnknownConnectionTypeException {
    //
    // int iConnectionType;
    //
    // if (_connection.m_connectionType ==
    // FrameworkConnectionType.PUSH_CONNECTION) {
    // iConnectionType = ConnectionDescription.PUSH_TO;
    // }
    // else if (_connection.m_connectionType ==
    // FrameworkConnectionType.PUSH_CONNECTION) {
    // iConnectionType = ConnectionDescription.PULL_FROM;
    // }
    // else {
    // throw new UnknownConnectionTypeException("unknown connection
    // type: "
    // + _connection.m_connectionType);
    // }
    //
    // JavaProcessDescription senderProc =
    // toManagedProcessDescription(_connection.m_sender);
    // senderProc.setIsLocal(false);
    // JavaProcessDescription receiverProc =
    // toManagedProcessDescription(_connection.m_receiver);
    // receiverProc.setIsLocal(true);
    // return new ConnectionDescription(senderProc, iConnectionType,
    // receiverProc);
    // }

}
