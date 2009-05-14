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

package cast.core.interfaces;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.pull.*;
import balt.core.data.FrameworkQuery;
import cast.cdl.TaskResult;

public interface GPRPullInterface {

    public interface GPRPullSender extends PullSender {

        public abstract void setPullConnector(String _connectionID,
                GPRPullConnectorOut _senderAdaptor);
    }

    public interface GPRPullReceiver extends PullReceiver {

        public abstract TaskResult receivePullQueryGPR(
                FrameworkQuery _query);
    }

    public interface GPRPullConnectorRegister extends
            PullConnectorRegister {

        public abstract void setPullReceiver(
                GPRPullReceiver _pr);
    }

    public interface GPRPullConnectorOut extends
            PullConnectorOut {

        public abstract TaskResult pull(FrameworkQuery _query)
                throws FrameworkConnectionException;
    }

    public interface GPRPullConnector extends
            GPRPullConnectorOut,
            GPRPullConnectorRegister, PullConnector {

    }

}
