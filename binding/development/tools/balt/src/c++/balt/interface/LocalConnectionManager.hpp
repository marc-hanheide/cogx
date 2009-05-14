/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * 
 * Copyright (C) 2006-2007 Nick Hawes, Gregor Berginc
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

#ifndef LOCAL_CONNECTION_MANAGER_H_
#define LOCAL_CONNECTION_MANAGER_H_

#include "includes.hpp"
//#include "processes/FrameworkProcesses.hpp"


typedef cast::StringMap<FrameworkProcess*>::map ProcessMap;
typedef std::vector < FrameworkConnector *> ConnectorVector;

//TODO Make this nicer!
#define PUSH_SENDER_NAME "PushSenderProcess"
#define PUSH_RECEIVER_NAME "PushReceiverProcess"
#define PULL_SENDER_NAME "PullSenderProcess"
#define PULL_RECEIVER_NAME "PullReceiverProcess"

/**
 * Class that manages connections between processes. "Local" is
 * actually incorrect now as it also handles connections with remote
 * objects too :/
 */
class LocalConnectionManager {

public:

    /** Default constructor. Connects to a naming service on "localhost"
        "1050" **/
    LocalConnectionManager();

    /** Constructor for specifying a CORBA naming service.
     *
     * @param _namingHost The host running the naming service.
     * @param _namingPort The port on which the service is running.
     */
    LocalConnectionManager(const std::string &_namingHost,
                           const std::string &_namingPort);

    /**
     * Desctructor. Stops all running processes and deletes them, then
     * deletes all local connection objects.
     */
    ~LocalConnectionManager();


    /**
     * Create a new C++ framework process.
     *
     *    @param _className The name of the class to create. See
     * PUSH_RECEIVER_NAME/PUSH_SENDER_NAME

     *@param _procName The unique name of this process.
     * @param Config information for this process.
     */
    void createProcess(const std::string &_className,
                       const std::string &_procName,
                       std::map<std::string,std::string> &_config);

    /**
     * Create a push connection between the two named processes (that
     * must have been created previously).
     *
     * @param _senderName The name of the push sender.
     * @param _receiverName The name of the push receiver.
     */
    void createPushConnection(const std::vector<std::string> &_senderNames,
                              const std::vector<std::string> &_receiverNames,
                              const std::string &_dataType,
                              const std::string &_connectionID);

    /**
     * Create a pull connection between the two named processes (that
     * must have been created previously).
     *
     * @param _senderName The name of the pull sender.
     * @param _receiverName The name of the pull receiver.
     */
    void createPullConnection(const std::vector<std::string> &_senderNames,
                              const std::string &_receiverName,
                              const std::string &_dataType,
                              const std::string &_connectionID);

    /**
     * Connect the named process as the sender in a remote push
     * connection.
     *
     * @param  _senderName The name of the push sender.
     * @param _remoteID The name of the server used for the connection.
     */
    void connectRemotePushConnectionSender(const std::string &_senderName,
                                           const std::string &_remoteID, 
					   const std::string &_dataType,
					   const std::string &_connectionID);



    /**
     * Connect the named process as the receiver in a remote push
     * connection.
     *
     * @param  _receiverName The name of the push receiver.
     * @param _remoteID The name of the server used for the connection.
     */
    void connectRemotePushConnectionReceiver(const std::string &_receiverName,
					     const std::string &_remoteID, 
					     const std::string &_dataType,
					     const std::string &_connectionID);

    /**
      * Connect the named process as the sender in a remote pull
      * connection.
      *
      * @param  _senderName The name of the pull sender.
      * @param _remoteID The name of the server used for the connection.
      */
    void connectRemotePullConnectionSender(const std::string &_senderName,
                                           const std::string &_remoteID, 
					   const std::string &_dataType,
					   const std::string &_connectionID);



    /**
     * Connect the named process as the receiver in a remote pull
     * connection.
     *
     * @param  _receiverName The name of the pull receiver.
     * @param _remoteID The name of the server used for the connection.
     */
    void connectRemotePullConnectionReceiver(const std::string &_receiverName,
					     const std::string &_remoteID, 
					     const std::string &_dataType,
					     const std::string &_connectionID);



    /**
     * Start the named process.
     *
     * @param _procName The process's unique name.
     */

    void startProcess(const std::string &_procName);

    /**
     * Run the named process.
     *
     * @param _procName The process's unique name.
     */

    void runProcess(const std::string &_procName);


    /**
     * Stop the named process.
     *
     * @param _procName The process's unique name.
     */
    void stopProcess(const std::string &_procName);


    static CORBA::ORB_var getORB() {return m_orb;};
    static CosNaming::NamingContext_var & getNamingContext() {return m_namingContext;};

private:


    /**
     * Print the available CORBA servers.
     */
    void printServers();

    //std::vector<PushConnectorImpl*> * m_pConnectors;

    ProcessMap * m_pProcesses;
    ConnectorVector * m_pConnectors;


    static CORBA::ORB_var m_orb;
    static CosNaming::NamingContext_var m_namingContext;
};

#endif
