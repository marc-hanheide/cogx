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
package balt.corba.impl;

import java.util.Properties;

import org.omg.CORBA.ORBPackage.InvalidName;
import org.omg.CosNaming.NameComponent;
import org.omg.CosNaming.NamingContextExt;
import org.omg.CosNaming.NamingContextExtHelper;
import org.omg.CosNaming.NamingContextPackage.CannotProceed;
import org.omg.CosNaming.NamingContextPackage.NotFound;
import org.omg.PortableServer.POA;
import org.omg.PortableServer.POAHelper;
import org.omg.PortableServer.POAManagerPackage.AdapterInactive;

import balt.corba.autogen.FrameworkBasics.FrameworkProcessManager;
import balt.corba.autogen.FrameworkBasics.FrameworkProcessManagerPOA;
import balt.corba.autogen.FrameworkBasics.FrameworkProcessManagerPOATie;
import balt.corba.autogen.FrameworkBasics.ProcessConnection;
import balt.core.connectors.ConnectionCreationException;
import balt.core.connectors.FrameworkConnectionException;
import balt.jni.NativeProcessLauncher;
import balt.management.ConnectionGraph;
import balt.management.FrameworkUtils;
import balt.management.InvalidConnectionProcessException;
import balt.management.ProcessLauncher;
import balt.management.ProcessLauncherException;

/**
 * @author nah
 */
public class FrameworkProcessManagerServer {

    private static Properties m_props;

    public static class FrameworkProcessManagerImpl
            extends
                FrameworkProcessManagerPOA {

        private boolean m_bDebug;

        /**
         * 
         */
        public FrameworkProcessManagerImpl(boolean _bDebug) {
            m_bDebug = _bDebug;
        }

        private ConnectionGraph m_graph;

        /*
         * (non-Javadoc)
         * 
         * @see balt.prototypes.sandbox.corba.autogen.FrameworkBasics.FrameworkProcessManagerOperations#createGraph(balt.prototypes.sandbox.corba.autogen.FrameworkBasics.ProcessConnection[])
         */
        public void createGraph(ProcessConnection[] _graph) {
            // System.out.println();
            System.out
                .println("Process Server: Creating balt processes... ");
            // System.out.println();

            NativeProcessLauncher.init(new String[0], m_props);

            // convert graph to more useful java objects
            m_graph = new ConnectionGraph();
            for (int i = 0; i < _graph.length; i++) {
                try {
                    // this extracts the class and works out where the
                    // processes should run
                  
                    m_graph.addConnection(ProcessLauncher
                        .toConnectionDescription(_graph[i]));

                }
                catch (Exception e) {
                    e.printStackTrace();
                    System.exit(1);
                }
            }

            if (m_bDebug) {
                System.out
                    .println("FrameworkProcessManagerImpl.createGraph()");
                System.out.println(m_graph);
                System.out.println();
            }

            // System.out.println("Creating processes...");

            try {
                ProcessLauncher.createGraphProcesses(m_graph);
            }
            catch (Exception e) {
                e.printStackTrace();
                System.exit(1);
            }
        }

        /*
         * (non-Javadoc)
         * 
         * @see balt.prototypes.sandbox.corba.autogen.FrameworkBasics.FrameworkProcessManagerOperations#createRemoteConnections()
         */
        public void createRemoteConnections() {

            // System.out.println();
            System.out
                .println("Process Server: Creating connection servers... ");
            // System.out.println();

            try {
                ProcessLauncher.createRemoteConnections(m_graph);
            }
            catch (InvalidConnectionProcessException e) {
                e.printStackTrace();
            }
            catch (ProcessLauncherException e) {
                e.printStackTrace();
            }
            catch (ConnectionCreationException e) {
                e.printStackTrace();
            }
        }

        /*
         * (non-Javadoc)
         * 
         * @see balt.prototypes.sandbox.corba.autogen.FrameworkBasics.FrameworkProcessManagerOperations#connectGraph()
         */
        public void connectGraph() {

            // System.out.println();
            System.out
                .println("Process Server: Connecting balt processes... ");
            // System.out.println();

            // TODO Rationalise all these exceptions as they all
            // basically mean the same thing!!!

            try {
                ProcessLauncher.connectGraph(m_graph);
            }
            catch (InvalidConnectionProcessException e) {
                e.printStackTrace();
            }
            catch (ProcessLauncherException e) {
                e.printStackTrace();
            }
            catch (ConnectionCreationException e) {
                e.printStackTrace();
            }
            catch (FrameworkConnectionException e) {
                e.printStackTrace();
            }
        }

        /*
         * (non-Javadoc)
         * 
         * @see balt.prototypes.sandbox.corba.autogen.FrameworkBasics.FrameworkProcessManagerOperations#runGraph()
         */
        public void startGraph() {
            // System.out.println();
            System.out
                .println("Process Server: Starting balt processes... ");

            ProcessLauncher.startGraph(m_graph);
        }

        /*
         * (non-Javadoc)
         * 
         * @see balt.prototypes.sandbox.corba.autogen.FrameworkBasics.FrameworkProcessManagerOperations#killGraph()
         */
        public void stopGraph() {
            // System.out.println();
            System.out
                .println("Process Server: Stopping balt processes... ");
            // System.out.println();

            ProcessLauncher.stopGraph(m_graph);
            System.out
            .println("Process Server: All processes stopped.");
            
            RemoteConnectionManager.getORB().shutdown(false);
        }

        /*
         * (non-Javadoc)
         * 
         * @see balt.corba.autogen.FrameworkBasics.FrameworkProcessManagerOperations#synchroniseWatches()
         */
        public void synchroniseWatches() {
            ProcessLauncher.resetClock();
        }

		public void runGraph() {
			 // System.out.println();
            System.out
                .println("Process Server: Running balt processes... ");

            ProcessLauncher.runGraph(m_graph);
			
		}

    }

    public static String serverNameForHost(String _hostname) {
        return SERVER_PREFIX + _hostname;
    }

    public static final String SERVER_PREFIX =
            "FrameworkProcessManager:";

    public static void main(String[] args) throws InvalidName,
            AdapterInactive,
            org.omg.CosNaming.NamingContextPackage.InvalidName,
            NotFound, CannotProceed {

        try {
            if (args.length >= 1) {

                // ////
                // Initialise ORB with naming host
                // ////

                System.out.println("naming host: " + args[0]);
                String namingHost = args[0];
                String namingPort = "1050";

                launchProcessServer(args, namingHost, namingPort, true);

            }
            else {
                System.out.println("only allowed is naming host");

            }
        }
        catch (org.omg.CORBA.COMM_FAILURE e) {
            System.err
                .println("\n\nCommunication failure. Is tnameserv running on "
                    + args[0] + "?");
        }
        catch (Exception e) {

            e.printStackTrace(System.out);
            System.err.println("ERROR: " + e.getClass());
        }

        System.out.println("FrameworkProcessManagerServer exiting ...");

    }

    public static void launchProcessServer(String[] _args,
                                           String _namingHost,
                                           String _namingPort,
                                           boolean _debug,
                                           String _serverName)
            throws InvalidName, AdapterInactive,
            org.omg.CosNaming.NamingContextPackage.InvalidName,
            NotFound, CannotProceed {
        m_props = new Properties();
        m_props.put("org.omg.CORBA.ORBInitialPort", _namingPort);
        m_props.put("org.omg.CORBA.ORBInitialHost", _namingHost);
        RemoteConnectionManager.init(_args, m_props);

        // ////
        // Get root POA
        // ////

        // Get reference to rootpoa & activate the POAManager
        POA rootPOA =
                POAHelper.narrow(RemoteConnectionManager.getORB()
                    .resolve_initial_references("RootPOA"));
        rootPOA.the_POAManager().activate();

        // ////
        // Get the naming service
        // ////

        // get the root naming context
        org.omg.CORBA.Object objRef =
                RemoteConnectionManager.getORB()
                    .resolve_initial_references("NameService");
        // Use NamingContextExt which is part of the
        // Interoperable
        // Naming Service specification.
        NamingContextExt ncRef = NamingContextExtHelper.narrow(objRef);

        // ////
        // Register the Process Manager with the RootPOA -- or
        // maybe create another?
        // ////

        // create servant and register it with the ORB
        FrameworkProcessManagerImpl fpmImpl =
                new FrameworkProcessManagerImpl(_debug);
        // create a tie, with servant being the delegate.
        FrameworkProcessManagerPOATie tie =
                new FrameworkProcessManagerPOATie(fpmImpl, rootPOA);
        // obtain the objectRef for the tie
        // this step also implicitly activates the
        // the object
        FrameworkProcessManager href =
                tie._this(RemoteConnectionManager.getORB());
        // bind the Object Reference in Naming
        String name;

        // if (args.length == 2) {
        // name = serverNameForHost(args[1]);
        // }
        // else {
        if (_serverName != null) {
            name = serverNameForHost(_serverName);
        }
        else {
            name = serverNameForHost(FrameworkUtils.getHostname());
        }
        // System.out.println("name at this point: " + name);
        // name =
        // serverNameForHost(FrameworkUtils.getHostIP());
        // helloImpl.m_givenName =
        // FrameworkUtils.getHostIP();
        // }

        // name = "FrameworkProcessManager";
        // helloImpl.m_givenName = "FrameworkProcessManager";

        NameComponent path[] = ncRef.to_name(name);

        // for (NameComponent nameComponent : path) {
        // System.out.println("nc: " + nameComponent.id);
        // }
        ncRef.rebind(path, href);

        // register adapter activator with rootPOA so that child
        // POAs can
        // be created on demand
        rootPOA
            .the_activator(new RemoteConnectionManager.RemoteConnectionManagerAdapterActivator());

        // wait for incoming requests
        System.out.println("Process Server: Ready and running....");

        RemoteConnectionManager.run();

    }

    /**
     * @param _args
     * @param _namingHost
     * @param _namingPort
     * @throws InvalidName
     * @throws AdapterInactive
     * @throws InvalidName
     * @throws NotFound
     * @throws CannotProceed
     */
    public static void launchProcessServer(String[] _args,
                                           String _namingHost,
                                           String _namingPort,
                                           boolean _debug)
            throws InvalidName, AdapterInactive,
            org.omg.CosNaming.NamingContextPackage.InvalidName,
            NotFound, CannotProceed {
        launchProcessServer(_args,_namingHost,_namingPort,_debug,null);
    }
}

// class PoaServantActivator extends LocalObject implements
// ServantActivator {
// // The incarnate operation is invoked by the POA whenever the POA
// receives
// // a request for an object that is not currently active, assuming the
// POA
// // has the RETAIN and USE_SERVANT_MANAGER policies
// public Servant incarnate(byte[] oid, POA adapter) throws
// ForwardRequest {
// try {
//            
// RemoteConnectionManager.RemotePushConnectorImpl servantObj = new
// RemoteConnectionManager.RemotePushConnectorImpl();
//            
// System.out.println("PoaServantActivator.incarnate(): Created \"" +
// servantObj.getClass().getName() + "\" " +
// "servant object for \"" + adapter.the_name() +
// "\"");
//            
// return servantObj;
// } catch (Exception e) {
// System.err.println("incarnate: Caught exception - " + e);
// }
// return null;
// }
//
// public void etherealize(byte[] oid, POA adapter, Servant serv,
// boolean cleanup_in_progress,
// boolean remaining_activations) {
// }
// }
