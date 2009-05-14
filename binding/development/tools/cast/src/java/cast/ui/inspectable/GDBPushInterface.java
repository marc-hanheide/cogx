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
package cast.ui.inspectable;

import balt.core.connectors.push.*;
import cast.cdl.guitypes.DrawBatch;

/**
 * @author mxz
 *
 */
public class GDBPushInterface
{

  public interface GDBPushConnector extends PushConnector,
          GDBPushInterface.GDBPushConnectorOut,
          GDBPushInterface.GDBPushConnectorRegister {

  }

  public interface GDBPushConnectorOut extends PushConnectorOut {

      /**
       * Push a data object on this connector.
       * 
       * @param _data
       *            The data to be pushed.
       */
      public abstract void push(String _src, DrawBatch _data);
  }

  public interface GDBPushConnectorRegister extends
          PushConnectorRegister {

      /**
       * Register a push receiver process with this connector.
       * 
       * @param _pr
       *            The receiver process that will receive pushes from
       *            this connector.
       */
      public abstract void registerPushReceiver(
              GDBPushInterface.GDBPushReceiver _pr);
  }

  public interface GDBPushReceiver extends PushReceiver {

      /**
       * Method to receive push data. When any push connector attached
       * to this process receives data it uses this method to push the
       * data into the process. The process must then use
       * FrameworkData.getSource() to determine where the information
       * came from.
       * 
       * @param _data
       *            The incoming push data.
       */
      public abstract void receivePushData(String _src, DrawBatch _data);
  }

  public interface GDBPushSender extends PushSender {

      /**
       * Set the push connector that this process will use for output.
       * NOTE: This is badly named because it implies only one
       * connector will be used.
       * 
       * @param _out
       *            The output connector for the process.
       */
      public abstract void setPushConnector(String _connectionID,
              GDBPushConnectorOut _out);

  }

}
