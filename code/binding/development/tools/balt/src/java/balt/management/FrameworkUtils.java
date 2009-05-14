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
package balt.management;

import java.lang.reflect.Method;
import java.net.*;
import java.util.*;

/**
 * Utility methods for the Framework.
 * 
 * @author nah
 */
public class FrameworkUtils {

    /**
     * IP address of localhost.
     */
    private static InetAddress m_localhost;

    private static ArrayList<InetAddress> m_localInterfaceAddress;

    /**
     * Host name of localhost.
     */
    private static String m_hostname;

    /**
     * IP string of localhost
     */
    private static String m_ip;

    static {
        try {

            Enumeration<NetworkInterface> networkInterfaces =
                    NetworkInterface.getNetworkInterfaces();

            m_localInterfaceAddress = new ArrayList<InetAddress>();

            while (networkInterfaces.hasMoreElements()) {
                NetworkInterface netface =
                        (NetworkInterface) networkInterfaces
                            .nextElement();

                // System.out.println("Net interface: "
                // + netface.getName());

                Enumeration e2 = netface.getInetAddresses();

                while (e2.hasMoreElements()) {
                    InetAddress ip = (InetAddress) e2.nextElement();

                    if (ip instanceof Inet4Address) {
                        m_localInterfaceAddress.add(ip);

                        // System.out.println("-------");
                        // System.out.println("link: "
                        // + ip.isLinkLocalAddress());
                        // System.out.println("any: "
                        // + ip.isAnyLocalAddress());
                        // System.out.println("loop: "
                        // + ip.isLoopbackAddress());
                        // System.out.println(ip.getCanonicalHostName());
                        // System.out.println(ip.getHostAddress());
                        // //
                        // System.out.println("-------");

                        if (!ip.isLinkLocalAddress()
                            && !ip.isLoopbackAddress()) {
                            setHostDetails(ip);
                        }

                    }
                }
            }

            // get localhost details
            m_localhost = InetAddress.getLocalHost();

            if (m_hostname == null) {
                // System.out.println("default");
                setHostDetails(m_localhost);
            }

            System.out.println("Process Server: Registered as: " + m_hostname);
            // System.out.println(m_ip);

        }
        catch (UnknownHostException e) {
            System.err
                .println("Unable to get local hostname. Exiting.");
            System.exit(1);
        }
        catch (SocketException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            System.exit(1);
        }
    }

    /**
     * @param ip
     */
    private static void setHostDetails(InetAddress ip) {

        String hostname = ip.getHostName();

        String first = hostname.split("\\.")[0];
        
        try {
            Integer.parseInt(first);
        }
        catch (NumberFormatException e) {
            //if the first bit can't be a number then set away...
            m_hostname = hostname;
            m_ip = ip.getHostAddress();
        }
        
    }

    /**
     * Find a method in the input class that returns a particular class.
     * The return class is compared to the input class using
     * "isAssignableFrom".
     * 
     * @param _cls
     *            The class to search for the method.
     * @param _returnClass
     *            The returning class for the method.
     * @param _methodName
     *            The name of the method to look for.
     * @param _paramLength
     *            The number of parameters for the method.
     * @return The found method or null.
     */
    public static Method findReturningMethod(Class<?> _cls,
                                             Class<?> _returnClass,
                                             String _methodName,
                                             int _paramLength) {
        try {
            // System.out.println(interfaces[i]);

            Method[] interfaceMethods = _cls.getMethods();

            for (int j = 0; j < interfaceMethods.length; j++) {

                if (!interfaceMethods[j].getName().equals(_methodName)) {
                    continue;
                }

                Class<?>[] p = interfaceMethods[j].getParameterTypes();

                if (p.length != _paramLength) {
                    continue;
                }

                Class<?> r = interfaceMethods[j].getReturnType();

                if (r.isAssignableFrom(_returnClass)) {
                    return interfaceMethods[j];
                }
            }

        }
        catch (SecurityException e) {
            e.printStackTrace();
            // System.exit(1);
        }
        return null;
    }

    /**
     * @param _cls
     * @param _returnClass
     * @param _methodPrefix
     * @param _paramLength
     * @return
     */
    public static Method findReturningMethodPrefix(Class<?> _cls,
                                                   Class<?> _returnClass,
                                                   String _methodPrefix,
                                                   int _paramLength) {
        try {
            // System.out.println(interfaces[i]);

            Method[] interfaceMethods = _cls.getMethods();

            for (int j = 0; j < interfaceMethods.length; j++) {

                if (!interfaceMethods[j].getName().startsWith(
                    _methodPrefix)) {
                    continue;
                }

                Class<?>[] p = interfaceMethods[j].getParameterTypes();

                if (p.length != _paramLength) {
                    continue;
                }

                Class<?> r = interfaceMethods[j].getReturnType();

                if (r.isAssignableFrom(_returnClass)) {
                    return interfaceMethods[j];
                }
            }

        }
        catch (SecurityException e) {
            e.printStackTrace();
            // System.exit(1);
        }
        return null;
    }

    public static Method findAssignableMethod(Class<?> _cls,
                                              Class<?> _assignmentClass,
                                              String _methodName,
                                              int _paramLength,
                                              int _assignmentParam) {
        try {
            // System.out.println(interfaces[i]);

            Method[] interfaceMethods = _cls.getMethods();

            for (int j = 0; j < interfaceMethods.length; j++) {

                if (!interfaceMethods[j].getName().equals(_methodName)) {
                    continue;
                }

                Class<?>[] p = interfaceMethods[j].getParameterTypes();

                if (p.length != _paramLength) {
                    continue;
                }

                if (p[_assignmentParam]
                    .isAssignableFrom(_assignmentClass)) {
                    return interfaceMethods[j];
                }
            }

        }
        catch (SecurityException e) {
            e.printStackTrace();
            // System.exit(1);
        }
        return null;
    }

    /**
     * Find an interface in the _implementingClass class that contains a
     * method with the name _methodName that has _paramLength
     * parameters, and that parameter _assignmentParam is assignable
     * from the class _assignmentClass.
     * 
     * @param _implementingClass
     *            The class to check for interfaces.
     * @param _assignmentClass
     *            The class that will be assigned to the method.
     * @param _methodName
     *            The method to seach for.
     * @param _paramLength
     *            The number of parameters the method should have.
     * @param _assignmentParam
     *            The parameter which will be used for assignment.
     * @return
     */
    public static Class<?> findAssignmentInterface(Class<?> _implementingClass,
                                                   Class<?> _assignmentClass,
                                                   String _methodName,
                                                   int _paramLength,
                                                   int _assignmentParam) {

        assert _assignmentParam < _paramLength : "Invalid parameter to check, it's greater than the specified number of parameters";

        // first try in implemented interfaces

        Class[] interfaces = _implementingClass.getInterfaces();
        for (int i = 0; i < interfaces.length; i++) {
            Method assignableMethod =
                    findAssignableMethod(interfaces[i],
                        _assignmentClass, _methodName, _paramLength,
                        _assignmentParam);
            if (assignableMethod != null) {
                return interfaces[i];
            }
        }

        // next call on super-classes... warning recursion ;)
        Class<?> superc = _implementingClass.getSuperclass();

        // if we're at object then we've gone too far!
        // if (superc.getClass().equals(Object.class)) {
        if (superc == null) {
            return null;
        }
        else {
            return findAssignmentInterface(superc, _assignmentClass,
                _methodName, _paramLength, _assignmentParam);
        }
    }

    /**
     * Find an interface in the _implementingClass class that contains a
     * method with the name _methodName that has _paramLength
     * parameters, and that parameter _assignmentParam is assignable
     * from the class _assignmentClass.
     * 
     * @param _implementingClass
     *            The class to check for interfaces.
     * @param _returnClass
     *            The class that will be assigned to the method.
     * @param _methodName
     *            The method to seach for.
     * @param _paramLength
     *            The number of parameters the method should have.
     * @param _assignmentParam
     *            The parameter which will be used for assignment.
     * @return
     */
    public static Class<?> findReturningInterface(Class<?> _implementingClass,
                                                  Class<?> _returnClass,
                                                  String _methodName,
                                                  int _paramLength) {

        Class[] interfaces = _implementingClass.getInterfaces();

        for (int i = 0; i < interfaces.length; i++) {
            Method returningMethod =
                    findReturningMethod(interfaces[i], _returnClass,
                        _methodName, _paramLength);
            if (returningMethod != null) {
                return interfaces[i];
            }
        }

        // next call on super-classes... warning recursion ;)
        Class<?> superc = _implementingClass.getSuperclass();

        // if we're at object then we've gone too far!
        // if (superc.getClass().equals(Object.class)) {
        if (superc == null) {
            return null;
        }
        else {
            return findReturningInterface(superc, _returnClass,
                _methodName, _paramLength);
        }

    }

    public static Class<?> findReturningInterfacePrefix(Class<?> _implementingClass,
                                                        Class<?> _returnClass,
                                                        String _methodPrefix,
                                                        int _paramLength) {

        Class[] interfaces = _implementingClass.getInterfaces();

        for (int i = 0; i < interfaces.length; i++) {
            Method returningMethod =
                    findReturningMethodPrefix(interfaces[i],
                        _returnClass, _methodPrefix, _paramLength);
            if (returningMethod != null) {
                return interfaces[i];
            }
        }

        // next call on super-classes... warning recursion ;)
        Class<?> superc = _implementingClass.getSuperclass();

        // if we're at object then we've gone too far!
        // if (superc.getClass().equals(Object.class)) {
        if (superc == null) {
            return null;
        }
        else {
            return findReturningInterfacePrefix(superc, _returnClass,
                _methodPrefix, _paramLength);
        }

    }

    public static String getHostname() {
        return m_hostname;
    }

    public static InetAddress getLocalhost() {
        return m_localhost;
    }

    public static String getHostIP() {
        return m_ip;
    }

    public static boolean areSameHost(String _a, String _b)
            throws UnknownHostException {
        if (_a.equals(_b)) {
            return true;
        }

        // bit ugly, but more convenient for current understanding
        if (_a.equals("localhost")) {
            InetAddress inetB = InetAddress.getByName(_b);
            return m_localInterfaceAddress.contains(inetB);
        }
        else if (_b.equals("localhost")) {
            InetAddress inetA = InetAddress.getByName(_a);
            return m_localInterfaceAddress.contains(inetA);
        }
        else {

            InetAddress inetA = InetAddress.getByName(_a);
            InetAddress inetB = InetAddress.getByName(_b);

            // System.out.println("comparing:\n" +
            // inetA.getHostAddress()
            // + "\n" + inetB.getHostAddress());

            if (inetA.getHostAddress().equals(inetB.getHostAddress())) {
                // System.out.println("same ip");
                return true;
            }
            else {
                return inetA.equals(inetB);
            }
        }
    }

}
