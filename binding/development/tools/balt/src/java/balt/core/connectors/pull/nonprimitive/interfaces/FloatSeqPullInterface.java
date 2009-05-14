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
package balt.core.connectors.pull.nonprimitive.interfaces;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.connectors.pull.*;
import balt.core.data.FrameworkQuery;

/**
 * @author nah
 */
public interface FloatSeqPullInterface {

    public interface FloatSeqPullConnector extends
            FloatSeqPullInterface.FloatSeqPullConnectorOut,
            FloatSeqPullInterface.FloatSeqPullConnectorRegister,
            PullConnector {

    }

    public interface FloatSeqPullConnectorOut extends PullConnectorOut {

        public abstract float[] pull(FrameworkQuery _query)
                throws FrameworkConnectionException;
    }

    public interface FloatSeqPullConnectorRegister extends
            PullConnectorRegister {

        public abstract void setPullReceiver(FloatSeqPullReceiver _pr);
    }

    public interface FloatSeqPullReceiver extends PullReceiver {

        public abstract float[] receivePullQueryFloatSeq(
                FrameworkQuery _query);
    }

    public interface FloatSeqPullSender extends PullSender {

        public abstract void setPullConnector(String _connectionID,
                FloatSeqPullConnectorOut _senderAdaptor);
    }

}
