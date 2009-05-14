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
package balt.management;

/**
 * A class that describes a connection between two sets of processes.
 * 
 * @author nah
 */
public class ConnectionDescription {

    /**
     * Constant to describe a push connection.
     */
    public static final int PUSH_TO = 0;
    
    /**
     * Constatnt to describe a pull connection.
     */
    public static final int PULL_FROM = 1;

    /**
     * First set of process descriptions.
     */
    private ManagedProcessDescription[] m_processA;

    /**
     * Second set of process descriptions.
     */
    private ManagedProcessDescription[] m_processB;

    /**
     * Type of connection.
     * @see m_processA m_processB
     */
    private int m_connectionType;
 
    
    private String m_dataType;
    
    private String m_id;

    private boolean isLocal;
    private boolean isSameLanguage;

    //    
    /**
     * @param _processA
     * @param _connectionType
     * @param _processB
     */
    public ConnectionDescription(ManagedProcessDescription[] _processA,
            int _connectionType, ManagedProcessDescription[] _processB,
            String _dataType, String _id) {

//        System.out
//            .println("ConnectionDescription.ConnectionDescription(): " + _id);
        
        // this should be checked previously anyway
        assert _processA.length > 0 : "no processes in sender list ";
        assert _processB.length > 0 : "no processes in receiver list ";

        m_processA = _processA;
        m_connectionType = _connectionType;
        m_processB = _processB;
        m_dataType = _dataType;
        m_id = _id;

        // determine whether connection is local
        isLocal = true;
        for (int i = 0; i < m_processA.length; i++) {
            if (!m_processA[i].isLocal()) {
                isLocal = false;
                break;
            }
        }
        if (isLocal) {
            for (int i = 0; i < m_processB.length; i++) {
                if (!m_processB[i].isLocal()) {
                    isLocal = false;
                    break;
                }
            }
        }

        // determine whether all processes are of the same language
        // ... do this by comparing derived classes of descriptions
        isSameLanguage = true;
        Class processClass = m_processA[0].getClass();
        for (int i = 1; i < m_processA.length; i++) {
            if (!m_processA[i].getClass().equals(processClass)) {
                isSameLanguage = false;
                break;
            }
        }
        if (isSameLanguage) {
            for (int i = 0; i < m_processB.length; i++) {
                if (!m_processB[i].getClass().equals(processClass)) {
                    isSameLanguage = false;
                    break;
                }
            }
        }

    }

    public int getConnectionType() {
        return m_connectionType;
    }

    public boolean isPushConnection() {
        return m_connectionType == PUSH_TO;
    }

    public boolean isPullConnection() {
        return m_connectionType == PULL_FROM;
    }

    public boolean isLocalConnection() {
        return isLocal;
    }

    public boolean isSameLanguageConnection() {
        // compare derived classes of descriptions
        return isSameLanguage;
    }

    public ManagedProcessDescription[] getProcessDescriptionA() {
        return m_processA;
    }

    public ManagedProcessDescription[] getProcessDescriptionB() {
        return m_processB;
    }

    private static String connectionToString(int _connectionType) {
        switch (_connectionType) {
            case PUSH_TO:
                return "push_to";
            case PULL_FROM:
                return "pull_from";
            default:
                return "unknown_connection";
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString() {
        char spacing = ' ';
        String ret = "[\"" + m_id + "\"" + spacing; 

        for (int i = 0; i < m_processA.length; i++) {
            ret += " [" + m_processA[i] + "]" + spacing;
        }

        ret += connectionToString(m_connectionType)  + spacing;

        for (int i = 0; i < m_processB.length; i++) {
            ret += " [" + m_processB[i] + "]" + spacing;
        }

        ret += "using datatype \"" + m_dataType + "\"]";

        return ret;
    }

    public String getDataType() {
        return m_dataType;
    }

    public String getId() {
        return m_id;
    }
}
