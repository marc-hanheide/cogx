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
import cast.cdl.WorkingMemoryChangeFilter;

public interface WMCFPullInterface {

    public interface WMCFPullSender extends PullSender {

        public abstract void setPullConnector(String _connectionID,
                WMCFPullConnectorOut _senderAdaptor);
    }

    public interface WMCFPullReceiver extends PullReceiver {

        public abstract WorkingMemoryChangeFilter receivePullQueryWMC(
                FrameworkQuery _query);
    }

    public interface WMCFPullConnectorRegister extends
            PullConnectorRegister {

        public abstract void setPullReceiver(
                WMCFPullReceiver _pr);
    }

    public interface WMCFPullConnectorOut extends
            PullConnectorOut {

        public abstract WorkingMemoryChangeFilter pull(FrameworkQuery _query)
                throws FrameworkConnectionException;
    }

    public interface WMCFPullConnector extends
            WMCFPullConnectorOut,
            WMCFPullConnectorRegister, PullConnector {

    }

}
