/*
 * @(#)IDLID.java 1.12 03/12/19 Copyright 2004 Sun Microsystems, Inc.
 * All rights reserved. SUN PROPRIETARY/CONFIDENTIAL. Use is subject to
 * license terms.
 */
/*
 * COMPONENT_NAME: idl.parser ORIGINS: 27 Licensed Materials - Property
 * of IBM 5639-D57 (C) COPYRIGHT International Business Machines Corp.
 * 1997, 1999 RMI-IIOP v1.0 US Government Users Restricted Rights - Use,
 * duplication or disclosure restricted by GSA ADP Schedule Contract
 * with IBM Corp. @(#)IDLID.java 1.12 03/12/19
 */

package org.cognitivesystems.idl;

import java.util.Iterator;
import java.util.StringTokenizer;

// NOTES:

public class IDLID extends RepositoryID {

    public IDLID() {
        _prefix = "";
        _name = "";
        _version = "1.0";
    } // ctor

    public IDLID(String prefix, String name, String version) {
        _prefix = prefix;
        _name = name;
        _version = version;
    } // ctor

    public String ID() {

        if (_prefix.equals("")) {
//            System.out.println("before IDL:" + _name + ':' + _version);
            String hackedName = removeDuplication(_name);
//            System.out.println(" after IDL:" + hackedName + ':'
//                + _version);
            return "IDL:" + hackedName + ':' + _version;
        }
        else {
            // System.out.println("2 IDL:" + _name + ':' + _version);
            return "IDL:" + _prefix + '/' + _name + ':' + _version;
        }
    } // ID

    /**
     * @param __name
     * @return
     */
    private String removeDuplication(String __name) {

        String name = "";
        StringTokenizer st = new StringTokenizer(_name, "//");
        String prev = "", next;
        while (st.hasMoreTokens()) {
            next = st.nextToken();

            // first one
            if (prev.equals("")) {
                name = next;
            }
            else {
                if (!prev.equals(next)) {
                    name += "/" + next;
                }
            }

            prev = next;

        }

        return name;

    }

    public String prefix() {
        return _prefix;
    } // prefix

    void prefix(String prefix) {
        if (prefix == null)
            _prefix = "";
        else
            _prefix = prefix;
    } // prefix

    public String name() {
        return _name;
    } // name

    void name(String name) {
        if (name == null)
            _name = "";
        else
            _name = name;
    } // name

    public String version() {
        return _version;
    } // version

    void version(String version) {
        if (version == null)
            _version = "";
        else
            _version = version;
    } // version

    void appendToName(String name) {

        // System.out.println(_name +" + " + name);

        if (name != null)
            if (_name.equals(""))
                _name = name;
            else
                _name = _name + '/' + name;

    } // appendToName

    void replaceName(String name) {
        if (name == null)
            _name = "";
        else {
            int index = _name.lastIndexOf('/');
            if (index < 0)
                _name = name;
            else
                _name = _name.substring(0, index + 1) + name;
        }

    } // replaceName

    public Object clone() {
        return new IDLID(_prefix, _name, _version);
    } // clone

    private String _prefix;
    private String _name;
    private String _version;
} // class IDLID
