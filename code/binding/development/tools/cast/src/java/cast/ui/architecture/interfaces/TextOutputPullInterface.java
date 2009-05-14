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
import cast.cdl.ui.TextOutput;

public interface TextOutputPullInterface {

    public interface TextOutputPullSender extends PullSender {

        public abstract void setPullConnector(String _connectionID,
                TextOutputPullConnectorOut _senderAdaptor);
    }

    public interface TextOutputPullReceiver extends PullReceiver {

        public abstract TextOutput receivePullQueryTextOutput(
                FrameworkQuery _query);
    }

    public interface TextOutputPullConnectorRegister extends
            PullConnectorRegister {

        public abstract void setPullReceiver(TextOutputPullReceiver _pr);
    }

    public interface TextOutputPullConnectorOut extends
            PullConnectorOut {

        public abstract TextOutput pull(FrameworkQuery _query)
                throws FrameworkConnectionException;
    }

    public interface TextOutputPullConnector extends
            TextOutputPullConnectorOut,
            TextOutputPullConnectorRegister, PullConnector {

    }

}
