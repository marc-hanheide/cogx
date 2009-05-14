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
package balt.clients;

import java.util.Properties;

import org.omg.CORBA.ORB;
import org.omg.CosNaming.NamingContext;
import org.omg.CosNaming.NamingContextHelper;
import org.omg.CosNaming.NamingContextPackage.*;

import balt.management.InvalidConnectionProcessException;



/**
 * @author nah
 */
public class FrameworkClient {

    /**
     * @param args
     * @throws InterruptedException
     * @throws org.omg.CORBA.ORBPackage.InvalidName
     * @throws InvalidName
     * @throws CannotProceed
     * @throws NotFound
     */
    public static void main(String[] args)
            throws InvalidConnectionProcessException,
            InterruptedException, NotFound, CannotProceed, InvalidName,
            org.omg.CORBA.ORBPackage.InvalidName {

        try {

            if (args.length == 1) {
                // Push Sender in C++

//                 ProcessDescription proc1Desc = new
//                 ProcessDescription();
                // proc1Desc.m_processName = "Push Sender 1";
                // // proc1Desc.m_className =
                // // NativeProcessLauncher.PUSH_SENDER_NAME;
                // // proc1Desc.m_language =
                // ProcessLanguage.CPP_PROCESS;
                // proc1Desc.m_className =
                // ProcessLauncher.PUSH_SENDER_NAME;
                // proc1Desc.m_language = ProcessLanguage.JAVA_PROCESS;
                // // proc1Desc.m_hostName = "louie";
                // // proc1Desc.m_hostName = "dewey";
                // proc1Desc.m_hostName = "waterhouse";
                // proc1Desc.m_configurationInfo = "dummy";
                //
                // ProcessDescription proc3Desc = new
                // ProcessDescription();
                // proc3Desc.m_processName = "Push Sender 2";
                // // proc3Desc.m_className =
                // // NativeProcessLauncher.PUSH_SENDER_NAME;
                // // proc3Desc.m_language =
                // ProcessLanguage.CPP_PROCESS;
                // proc3Desc.m_className =
                // ProcessLauncher.PUSH_SENDER_NAME;
                // proc3Desc.m_language = ProcessLanguage.JAVA_PROCESS;
                // // proc3Desc.m_hostName = "louie";
                // // proc3Desc.m_hostName = "dewey";
                // proc3Desc.m_hostName = "waterhouse";
                // proc3Desc.m_configurationInfo = "dummy";
                //
                // // Push Receiver in Java
                //
                // ProcessDescription proc2Desc = new
                // ProcessDescription();
                // proc2Desc.m_processName = "Push Receiver 1";
                // proc2Desc.m_className =
                // ProcessLauncher.PUSH_RECEIVER_NAME;
                // proc2Desc.m_language = ProcessLanguage.JAVA_PROCESS;
                // // proc2Desc.m_className =
                // // NativeProcessLauncher.PUSH_RECEIVER_NAME;
                // // proc2Desc.m_language =
                // ProcessLanguage.CPP_PROCESS;
                // // proc2Desc.m_hostName = "dewey";
                // // proc2Desc.m_hostName = "louie";
                // proc2Desc.m_hostName = "waterhouse";
                // proc2Desc.m_configurationInfo = "dummy";
                //
                // ProcessDescription proc4Desc = new
                // ProcessDescription();
                // proc4Desc.m_processName = "Push Receiver 2";
                // proc4Desc.m_className =
                // ProcessLauncher.PUSH_RECEIVER_NAME;
                // proc4Desc.m_language = ProcessLanguage.JAVA_PROCESS;
                // // proc4Desc.m_className =
                // // NativeProcessLauncher.PUSH_RECEIVER_NAME;
                // // proc4Desc.m_language =
                // ProcessLanguage.CPP_PROCESS;
                // // proc4Desc.m_hostName = "dewey";
                // // proc4Desc.m_hostName = "louie";
                // proc4Desc.m_hostName = "waterhouse";
                // proc4Desc.m_configurationInfo = "dummy";
                //
                // // ProcessConnection[] connections = new
                // // ProcessConnection[2];
                // // connections[0] = new ProcessConnection(proc1Desc,
                // // proc2Desc,
                // FrameworkConnectionType.PUSH_CONNECTION,
                // // "String", "String Push Connection");
                // // connections[1] = new ProcessConnection(proc1Desc,
                // // proc2Desc,
                // FrameworkConnectionType.PUSH_CONNECTION,
                // // "LongSeq", "LongSeq Push Connection");
                //
                // ProcessConnection[] connections = new
                // ProcessConnection[1];
                // connections[0] = new ProcessConnection(
                // new ProcessDescription[] {
                // proc1Desc, proc3Desc
                // }, new ProcessDescription[] {
                // proc2Desc, proc4Desc
                // }, FrameworkConnectionType.PUSH_CONNECTION, "Int",
                // "Int Push Connection");

                // // Pushes in C++
                // ProcessDescription proc1Desc = new
                // ProcessDescription();
                // proc1Desc.m_processName = "Push Sender 1";
                // proc1Desc.m_className =
                // NativeProcessLauncher.PUSH_SENDER_NAME;
                // proc1Desc.m_language = ProcessLanguage.CPP_PROCESS;
                // // proc1Desc.m_hostName = "louie";
                // // proc1Desc.m_hostName = "dewey";
                // proc1Desc.m_hostName = "waterhouse";
                // proc1Desc.m_configurationInfo = "dummy";
                //
                // ProcessDescription proc3Desc = new
                // ProcessDescription();
                // proc3Desc.m_processName = "Push Sender 2";
                // proc3Desc.m_className =
                // NativeProcessLauncher.PUSH_SENDER_NAME;
                // proc3Desc.m_language = ProcessLanguage.CPP_PROCESS;
                // // proc3Desc.m_hostName = "louie";
                // // proc3Desc.m_hostName = "dewey";
                // proc3Desc.m_hostName = "waterhouse";
                // proc3Desc.m_configurationInfo = "dummy";
                //
                // ProcessDescription proc2Desc = new
                // ProcessDescription();
                // proc2Desc.m_processName = "Push Receiver 1";
                // proc2Desc.m_className =
                // NativeProcessLauncher.PUSH_RECEIVER_NAME;
                // proc2Desc.m_language = ProcessLanguage.CPP_PROCESS;
                // // proc2Desc.m_hostName = "dewey";
                // // proc2Desc.m_hostName = "louie";
                // proc2Desc.m_hostName = "waterhouse";
                // proc2Desc.m_configurationInfo = "dummy";
                //
                // ProcessDescription proc4Desc = new
                // ProcessDescription();
                // proc4Desc.m_processName = "Push Receiver 2";
                // proc4Desc.m_className =
                // NativeProcessLauncher.PUSH_RECEIVER_NAME;
                // proc4Desc.m_language = ProcessLanguage.CPP_PROCESS;
                // // proc4Desc.m_hostName = "dewey";
                // // proc4Desc.m_hostName = "louie";
                // proc4Desc.m_hostName = "waterhouse";
                // proc4Desc.m_configurationInfo = "dummy";
                //
                // ProcessConnection[] connections = new
                // ProcessConnection[1];
                // connections[0] = new ProcessConnection(
                // new ProcessDescription[] {
                // proc1Desc, proc3Desc
                // }, new ProcessDescription[] {
                // proc2Desc, proc4Desc
                // }, FrameworkConnectionType.PUSH_CONNECTION, "Int",
                // "Int Push Connection");

                // // Pushes Java/C++
//                ProcessDescription proc1Desc = new
//                ProcessDescription();
//                proc1Desc.m_processName = "Push Sender 1";
//                proc1Desc.m_className =
//                ProcessLauncher.PUSH_SENDER_NAME;
//                proc1Desc.m_language = ProcessLanguage.JAVA_PROCESS;
//                // proc1Desc.m_hostName = "louie";
//                 proc1Desc.m_hostName = "dewey";
//                //proc1Desc.m_hostName = "waterhouse";
//                proc1Desc.m_configurationInfo = "dummy";
//                
//                ProcessDescription proc3Desc = new
//                ProcessDescription();
//                proc3Desc.m_processName = "Push Sender 2";
//                proc3Desc.m_className =
//                NativeProcessLauncher.PUSH_SENDER_NAME;
//                proc3Desc.m_language = ProcessLanguage.CPP_PROCESS;
//                // proc3Desc.m_className =
//                // ProcessLauncher.PUSH_SENDER_NAME;
//                // proc3Desc.m_language =
//                // ProcessLanguage.JAVA_PROCESS;
//                 proc3Desc.m_hostName = "louie";
//                // proc3Desc.m_hostName = "dewey";
//                //proc3Desc.m_hostName = "waterhouse";
//                proc3Desc.m_configurationInfo = "dummy";
//                
//                ProcessDescription proc2Desc = new
//                ProcessDescription();
//                proc2Desc.m_processName = "Push Receiver 1";
//                proc2Desc.m_className =
//                NativeProcessLauncher.PUSH_RECEIVER_NAME;
//                proc2Desc.m_language = ProcessLanguage.CPP_PROCESS;
//                // proc2Desc.m_hostName = "dewey";
//                 proc2Desc.m_hostName = "louie";
//		 //proc2Desc.m_hostName = "waterhouse";
//                proc2Desc.m_configurationInfo = "dummy";
//                
//                ProcessDescription proc4Desc = new
//                ProcessDescription();
//                proc4Desc.m_processName = "Push Receiver 2";
//                proc4Desc.m_className =
//                ProcessLauncher.PUSH_RECEIVER_NAME;
//                proc4Desc.m_language = ProcessLanguage.JAVA_PROCESS;
//                // proc4Desc.m_className =
//                // NativeProcessLauncher.PUSH_RECEIVER_NAME;
//                // proc4Desc.m_language =
//                // ProcessLanguage.CPP_PROCESS;
//                 proc4Desc.m_hostName = "dewey";
//                // proc4Desc.m_hostName = "louie";
//                //proc4Desc.m_hostName = "waterhouse";
//                proc4Desc.m_configurationInfo = "dummy";
//                
//                ProcessConnection[] connections = new
//                ProcessConnection[1];
//                connections[0] = new ProcessConnection(
//                new ProcessDescription[] {
//                proc1Desc, proc3Desc
//                }, new ProcessDescription[] {
//                proc2Desc, proc4Desc
//                }, FrameworkConnectionType.PUSH_CONNECTION, "Int",
//                "Int Push Connection");

                // Pulls in Java

                // ProcessDescription proc1Desc = new
                // ProcessDescription();
                // proc1Desc.m_processName = "Pull Sender 1";
                // // proc1Desc.m_className =
                // // NativeProcessLauncher.Pull_SENDER_NAME;
                // // proc1Desc.m_language =
                // ProcessLanguage.CPP_PROCESS;
                // proc1Desc.m_className =
                // ProcessLauncher.PULL_SENDER_NAME;
                // proc1Desc.m_language = ProcessLanguage.JAVA_PROCESS;
                // // proc1Desc.m_hostName = "louie";
                // // proc1Desc.m_hostName = "dewey";
                // proc1Desc.m_hostName = "waterhouse";
                // proc1Desc.m_configurationInfo = "dummy";
                //
                // ProcessDescription proc3Desc = new
                // ProcessDescription();
                // proc3Desc.m_processName = "Pull Sender 2";
                // // proc3Desc.m_className =
                // // NativeProcessLauncher.Pull_SENDER_NAME;
                // // proc3Desc.m_language =
                // ProcessLanguage.CPP_PROCESS;
                // proc3Desc.m_className =
                // ProcessLauncher.PULL_SENDER_NAME;
                // proc3Desc.m_language = ProcessLanguage.JAVA_PROCESS;
                // // proc3Desc.m_hostName = "louie";
                // // proc3Desc.m_hostName = "dewey";
                // proc3Desc.m_hostName = "waterhouse";
                // proc3Desc.m_configurationInfo = "dummy";
                //
                // // Pull Receiver in Java
                //
                // ProcessDescription proc2Desc = new
                // ProcessDescription();
                // proc2Desc.m_processName = "Pull Receiver 1";
                // proc2Desc.m_className =
                // ProcessLauncher.PULL_RECEIVER_NAME;
                // proc2Desc.m_language = ProcessLanguage.JAVA_PROCESS;
                // // proc2Desc.m_className =
                // // NativeProcessLauncher.Pull_RECEIVER_NAME;
                // // proc2Desc.m_language =
                // ProcessLanguage.CPP_PROCESS;
                // // proc2Desc.m_hostName = "dewey";
                // // proc2Desc.m_hostName = "louie";
                // proc2Desc.m_hostName = "waterhouse";
                // proc2Desc.m_configurationInfo = "dummy";

                //
                // ProcessConnection[] connections = new
                // ProcessConnection[1];
                // connections[0] = new ProcessConnection(
                // new ProcessDescription[] {
                // proc1Desc, proc3Desc
                // }, new ProcessDescription[] {
                // proc2Desc
                // }, FrameworkConnectionType.PULL_CONNECTION, "Int",
                // "Int Pull Connection");

                // Pulls in C++

                // ProcessDescription proc1Desc = new
                // ProcessDescription();
                // proc1Desc.m_processName = "Pull Sender 1";
                // proc1Desc.m_className =
                // NativeProcessLauncher.PULL_SENDER_NAME;
                // proc1Desc.m_language = ProcessLanguage.CPP_PROCESS;
                // // proc1Desc.m_hostName = "louie";
                // // proc1Desc.m_hostName = "dewey";
                // proc1Desc.m_hostName = "waterhouse";
                // proc1Desc.m_configurationInfo = "dummy";
                //
                // ProcessDescription proc3Desc = new
                // ProcessDescription();
                // proc3Desc.m_processName = "Pull Sender 2";
                // proc3Desc.m_className =
                // NativeProcessLauncher.PULL_SENDER_NAME;
                // proc3Desc.m_language = ProcessLanguage.CPP_PROCESS;
                // // proc3Desc.m_hostName = "louie";
                // // proc3Desc.m_hostName = "dewey";
                // proc3Desc.m_hostName = "waterhouse";
                // proc3Desc.m_configurationInfo = "dummy";
                //
                // ProcessDescription proc2Desc = new
                // ProcessDescription();
                // proc2Desc.m_processName = "Pull Receiver 1";
                // proc2Desc.m_className =
                // NativeProcessLauncher.PULL_RECEIVER_NAME;
                // proc2Desc.m_language = ProcessLanguage.CPP_PROCESS;
                // // proc2Desc.m_hostName = "dewey";
                // // proc2Desc.m_hostName = "louie";
                // proc2Desc.m_hostName = "waterhouse";
                // proc2Desc.m_configurationInfo = "dummy";
                //
                // ProcessConnection[] connections = new
                // ProcessConnection[1];
                // connections[0] = new ProcessConnection(
                // new ProcessDescription[] {
                // proc1Desc, proc3Desc
                // }, new ProcessDescription[] {
                // proc2Desc
                // }, FrameworkConnectionType.PULL_CONNECTION, "Int",
                // "Int Pull Connection");

                // Pulls in Java/C++

//                 ProcessDescription proc1Desc = new ProcessDescription();
//                 proc1Desc.m_processName = "Pull Sender 1";
//                 proc1Desc.m_className = ProcessLauncher.PULL_SENDER_NAME;
//                 proc1Desc.m_language = ProcessLanguage.JAVA_PROCESS;
//                 // proc1Desc.m_className =
//                 // NativeProcessLauncher.PULL_SENDER_NAME;
//                 // proc1Desc.m_language =
//                 // ProcessLanguage.CPP_PROCESS;
//                 // proc1Desc.m_hostName = "louie";
//                  proc1Desc.m_hostName = "dewey";
//                 //proc1Desc.m_hostName = "waterhouse";
//                 proc1Desc.m_configurationInfo = "dummy";

//                 ProcessDescription proc3Desc = new ProcessDescription();
//                 proc3Desc.m_processName = "Pull Sender 2";
//                 proc3Desc.m_className = NativeProcessLauncher.PULL_SENDER_NAME;
//                 proc3Desc.m_language = ProcessLanguage.CPP_PROCESS;
// 		proc3Desc.m_hostName = "louie";
//                 // proc3Desc.m_hostName = "dewey";
//                 //proc3Desc.m_hostName = "waterhouse";
//                 proc3Desc.m_configurationInfo = "dummy";

//                 ProcessDescription proc2Desc = new ProcessDescription();
//                 proc2Desc.m_processName = "Pull Receiver 1";
//                 proc2Desc.m_className = ProcessLauncher.PULL_RECEIVER_NAME;
//                 proc2Desc.m_language = ProcessLanguage.JAVA_PROCESS;
//                 // proc2Desc.m_className =
//                 // NativeProcessLauncher.PULL_RECEIVER_NAME;
//                 // proc2Desc.m_language =
//                 // ProcessLanguage.CPP_PROCESS;
//                  proc2Desc.m_hostName = "dewey";
//                 // proc2Desc.m_hostName = "louie";
//                 //proc2Desc.m_hostName = "waterhouse";
//                 proc2Desc.m_configurationInfo = "dummy";

//                 ProcessConnection[] connections = new ProcessConnection[1];
//                 connections[0] = new ProcessConnection(
//                     new ProcessDescription[] {
//                         proc1Desc, proc3Desc
//                     }, new ProcessDescription[] {
//                         proc2Desc
//                     }, FrameworkConnectionType.PULL_CONNECTION, "Int",
//                     "Int Pull Connection");

//                System.out.println("naming host: " + args[0]);
//
//                ClientUtils.runFramework(args[0], "1050", connections,
//                    5000);

            }
            else if (args.length == 2) {
                System.out.println("naming host: " + args[0]);

                Properties props = new Properties();
                props.put("org.omg.CORBA.ORBInitialPort", "1050");
                props.put("org.omg.CORBA.ORBInitialHost", args[0]);
                ORB orb = ORB.init(args, props);

                NamingContext nc = NamingContextHelper.narrow(orb
                    .resolve_initial_references("NameService"));

                ClientUtils.browseNaming(orb, nc);

            }
            else {
                System.err.println("wrong args");
            }

        }
        catch (Exception e) {
            System.out.println("ERROR: " + e);
            e.printStackTrace();
        }
    }
}
