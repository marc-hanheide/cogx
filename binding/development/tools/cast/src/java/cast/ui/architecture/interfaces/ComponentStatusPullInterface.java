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

package cast.ui.architecture.interfaces;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.pull.*;
import balt.core.data.FrameworkQuery;
import cast.cdl.ui.ComponentStatus;

public interface ComponentStatusPullInterface {

    public interface ComponentStatusPullSender extends PullSender {

        public abstract void setPullConnector(String _connectionID,
                ComponentStatusPullConnectorOut _senderAdaptor);
    }

    public interface ComponentStatusPullReceiver extends PullReceiver {

        public abstract ComponentStatus receivePullQueryComponentStatus(
                FrameworkQuery _query);
    }

    public interface ComponentStatusPullConnectorRegister extends
            PullConnectorRegister {

        public abstract void setPullReceiver(ComponentStatusPullReceiver _pr);
    }

    public interface ComponentStatusPullConnectorOut extends
            PullConnectorOut {

        public abstract ComponentStatus pull(FrameworkQuery _query)
                throws FrameworkConnectionException;
    }

    public interface ComponentStatusPullConnector extends
            ComponentStatusPullConnectorOut,
            ComponentStatusPullConnectorRegister, PullConnector {

    }

}
