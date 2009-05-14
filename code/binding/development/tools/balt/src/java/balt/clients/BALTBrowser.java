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
import org.omg.CORBA.ORBPackage.InvalidName;
import org.omg.CosNaming.NamingContextExt;
import org.omg.CosNaming.NamingContextExtHelper;
import org.omg.CosNaming.NamingContextPackage.*;

import balt.corba.impl.RemoteConnectionManager;

/**
 * @author nah
 */
public class BALTBrowser {

    /**
     * @param args
     * @throws InvalidName
     * @throws CannotProceed
     * @throws NotFound
     * @throws InvalidName
     * @throws org.omg.CosNaming.NamingContextPackage.InvalidName
     * @throws ArchitectureConfigurationException
     */
    public static void main(String[] args) throws NotFound,
            CannotProceed, InvalidName, InvalidName,
            org.omg.CosNaming.NamingContextPackage.InvalidName {

        String namingHost = null;

        try {
            for (int i = 0; i < args.length; i++) {
                if (args[i].equals("-h")) {
                    namingHost = args[i + 1];
                }
            }
        }
        catch (ArrayIndexOutOfBoundsException e) {
            showArgs();
            return;
        }

        if (namingHost == null) {
            showArgs();
            return;
        }

        System.out.println("naming host: " + namingHost);

        // Store input properties
        Properties props = new Properties();
        props.put("org.omg.CORBA.ORBInitialPort", "1050");
        props.put("org.omg.CORBA.ORBInitialHost", namingHost);

        // initialise orb
        RemoteConnectionManager.init(args, props);

        ORB orb = RemoteConnectionManager.getORB();
        // first task is to find the object to connect to
        // get the root naming context
        org.omg.CORBA.Object objRef = orb
            .resolve_initial_references("NameService");

        // Use NamingContextExt instead of NamingContext. This is
        // part of the Interoperable naming Service.
        NamingContextExt ncRef = NamingContextExtHelper.narrow(objRef);

        ClientUtils.browseNaming(orb, ncRef);
    }

    /**
     * 
     */
    private static void showArgs() {
        System.err
            .println("BALTBrowser arguments:\n\t -h naming host REQUIRED\n");
    }

}
