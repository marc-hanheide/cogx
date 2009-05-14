/*
 * CAST - The CoSy Architecture Schema Toolkit
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
package cast.server;

import balt.corba.autogen.FrameworkBasics.ProcessConnection;
import cast.configuration.*;
import cast.core.CASTUtils;

/**
 * @author nah
 */
public class CASTCommandLineClient {

    /**
     * @param args
     * @throws ArchitectureConfigurationException
     */
    public static void main(String[] args)
            throws ArchitectureConfigurationException {

        String namingHost = null;
        String configFile = null;
        long runTime = 0;

        try {
            for (int i = 0; i < args.length; i++) {
                if (args[i].equals("-f")) {
                    configFile = args[i + 1];
                }
                else if (args[i].equals("-h")) {
                    namingHost = args[i + 1];
                }
                else if (args[i].equals("-r")) {
                    try {
                        runTime = Long.parseLong(args[i + 1]);
                    }
                    catch (NumberFormatException e) {
                        e.printStackTrace();
                        System.err
                            .println("Incorrect runtime specification: "
                                + args[i + 1]);
                    }
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

        if (configFile == null) {
            showArgs();
            return;
        }

        CASTConnectionConfiguration ccc = CASTConfigParser
            .parseConfigFile(configFile);

        ProcessConnection[] extras = CASTConfigParser.parseExtras(configFile);
        
        CASTUtils.runCAST(namingHost, ccc, extras, runTime);
    }

    /**
     * 
     */
    private static void showArgs() {
        System.err
            .println("CAATCommandLineClient arguments:\n\t -h naming host REQUIRED\n\t -f config file REQUIRED\n\t -r millisecond run time OPTIONAL");
    }

}
