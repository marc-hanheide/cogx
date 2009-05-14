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
package cast.ui.architecture;

import java.util.ArrayList;
import java.util.Properties;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.data.FrameworkQuery;
import balt.core.processes.FrameworkProcess;
import cast.configuration.*;
import cast.cdl.ui.*;
import cast.ui.UIUtils;
import cast.ui.architecture.interfaces.ArchUIServer;
import cast.ui.architecture.interfaces.ComponentStatusPullInterface.ComponentStatusPullConnectorOut;

/**
 * The object that receives and handles UI connections from caat
 * components.
 * 
 * @author nah
 */
public abstract class UIServer extends FrameworkProcess implements
        ArchUIServer {

    private ArrayList<ComponentStatusPullConnectorOut> m_connection;

    private FrameworkQuery m_queryObject;
    private long m_updateSleep;
    private ArchitectureConfiguration m_connectionConfiguration;
    /**
     * @param _id
     */
    public UIServer(String _id) {
        super(_id);
        m_connection = new ArrayList<ComponentStatusPullConnectorOut>(
            10);
        m_queryObject = new FrameworkQuery(_id, "");
        UIUtils.init();
        setUpdateRate(20);
    }

    /**
     * @param _src
     * @param _data
     */
    protected abstract void handleComponentEvent(String _src, ComponentEvent _data);

    /**
     * @param _src
     * @param _data
     */
    protected abstract void handleTextOutput(String _src,
            TextOutput _data);

    /**
     * @param _connectionConfiguration
     */
    protected abstract void setArchitectureConfiguration(
            ArchitectureConfiguration _connectionConfiguration);

    /**
     * @param _componentStatus
     * @param _conn
     */
    protected abstract void setupComponentConnection(
            ComponentStatus _componentStatus,
            ComponentStatusPullConnectorOut _conn);

    protected void setUpdateRate(int m_hertz) {
        m_updateSleep = 1000 / m_hertz;
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.processes.FrameworkProcess#configure(java.util.Properties)
     */
    @Override
    public void configure(Properties _config) {
        if (_config.containsKey(CASTConfigParser.CONFIG_FILE_FLAG)) {
            try {
                m_connectionConfiguration = CASTConfigParser
                    .parseConfigString(_config
                        .getProperty(CASTConfigParser.CONFIG_FILE_FLAG));

                // pass arch info onto derived class
                setArchitectureConfiguration(m_connectionConfiguration);
            }
            catch (ArchitectureConfigurationException e) {
                e.printStackTrace();
                System.exit(1);
            }
        }
    }

    public void receivePushData(String _src, ComponentEvent _data) {
//        System.out.println(UIUtils.toString(_data));
        handleComponentEvent(_src, _data);
    }

    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.interfaces.TextOutputPushInterface.TextOutputPushReceiver#receivePushData(java.lang.String,
     *      caat.corba.autogen.CAAT.ui.TextOutput)
     */
    public void receivePushData(String _src, TextOutput _data) {
        // System.out.println("UI\t" + _src + ": " + _data.m_string);
        handleTextOutput(_src, _data);
    }

    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.interfaces.LogObjectPullInterface.LogObjectPullSender#setPullConnector(java.lang.String,
     *      caat.ui.architecture.interfaces.LogObjectPullInterface.LogObjectPullConnectorOut)
     */
    public void setPullConnector(String _connectionID,
            ComponentStatusPullConnectorOut _connector) {
//        System.out.println(_connectionID);
        m_connection.add(_connector);
    }

    @Override
    public void start() {
        super.start();

        // connect up dervived class with components
        for (ComponentStatusPullConnectorOut conn : m_connection) {
            try {
                ComponentStatus componentStatus = conn
                    .pull(m_queryObject);
                    setupComponentConnection(componentStatus, conn);
            }
            catch (FrameworkConnectionException e) {
                e.printStackTrace();
                System.exit(1);
            }
        }

        // now clear connections, as we no longer require them
        m_connection.clear();
    }

}
