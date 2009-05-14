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
package balt.core.connectors;

import java.util.HashMap;

import balt.core.connectors.pull.PullConnector;
import balt.core.connectors.pull.PullReceiver;
import balt.core.connectors.pull.nonprimitive.interfaces.*;
import balt.core.connectors.pull.primitive.interfaces.*;
import balt.core.connectors.push.*;
import balt.core.connectors.push.nonprimitive.interfaces.*;
import balt.core.connectors.push.primitive.interfaces.*;
import balt.core.data.FrameworkLocalData;
import balt.core.data.FrameworkQuery;


/**
 * Handcoded for now. In the future it will probably be easier to
 * auto-generate connectors using the stuff here...
 * http://java.sun.com/products/jfc/tsc/articles/generic-listener/index.html
 * 
 * @author nah
 */
public class LocalConnectionFactory {

    public static interface PushConnectionCreator {

        abstract PushConnector connect(PushSender _send,
                PushReceiver _recv, String _id);
    };

    private static HashMap<String, Class> m_dataTypeMap;
    private static HashMap<Class, Class> m_pushConnectorMap;
    private static HashMap<Class, Class> m_pullConnectorMap;

    static {
        m_dataTypeMap = new HashMap<String, Class>();
        m_pushConnectorMap = new HashMap<Class, Class>();
        m_pullConnectorMap = new HashMap<Class, Class>();

        addDatatype("string", String.class,
            StringPushInterface.StringPushConnector.class,
            StringPullInterface.StringPullConnector.class);

        addDatatype("longlong", long.class,
            LongPushInterface.LongPushConnector.class,
            LongPullInterface.LongPullConnector.class);

        addDatatype("short", short.class,
            ShortPushInterface.ShortPushConnector.class,
            ShortPullInterface.ShortPullConnector.class);

        addDatatype("int", int.class,
            IntPushInterface.IntPushConnector.class,
            IntPullInterface.IntPullConnector.class);

        addDatatype("long", int.class,
            IntPushInterface.IntPushConnector.class,
            IntPullInterface.IntPullConnector.class);

        addDatatype("double", double.class,
            DoublePushInterface.DoublePushConnector.class,
            DoublePullInterface.DoublePullConnector.class);

        addDatatype("float", float.class,
            FloatPushInterface.FloatPushConnector.class,
            FloatPullInterface.FloatPullConnector.class);

        addDatatype("char", char.class,
            CharPushInterface.CharPushConnector.class,
            CharPullInterface.CharPullConnector.class);

        addDatatype("byte", byte.class,
            BytePushInterface.BytePushConnector.class,
            BytePullInterface.BytePullConnector.class);

        addDatatype("boolean", boolean.class,
            BoolPushInterface.BoolPushConnector.class,
            BoolPullInterface.BoolPullConnector.class);

        addDatatype("bool", boolean.class,
            BoolPushInterface.BoolPushConnector.class,
            BoolPullInterface.BoolPullConnector.class);

        addDatatype("stringseq", String[].class,
            StringSeqPushInterface.StringSeqPushConnector.class,
            StringSeqPullInterface.StringSeqPullConnector.class);

        addDatatype("intseq", int[].class,
            IntSeqPushInterface.IntSeqPushConnector.class,
            IntSeqPullInterface.IntSeqPullConnector.class);

        addDatatype("longseq", long[].class,
            LongSeqPushInterface.LongSeqPushConnector.class,
            LongSeqPullInterface.LongSeqPullConnector.class);

        addDatatype("shortseq", short[].class,
            ShortSeqPushInterface.ShortSeqPushConnector.class,
            ShortSeqPullInterface.ShortSeqPullConnector.class);

        addDatatype("doubleseq", double[].class,
            DoubleSeqPushInterface.DoubleSeqPushConnector.class,
            DoubleSeqPullInterface.DoubleSeqPullConnector.class);

        addDatatype("floatseq", float[].class,
            FloatSeqPushInterface.FloatSeqPushConnector.class,
            FloatSeqPullInterface.FloatSeqPullConnector.class);

        addDatatype("charseq", char[].class,
            CharSeqPushInterface.CharSeqPushConnector.class,
            CharSeqPullInterface.CharSeqPullConnector.class);

        addDatatype("byteseq", byte[].class,
            ByteSeqPushInterface.ByteSeqPushConnector.class,
            ByteSeqPullInterface.ByteSeqPullConnector.class);

        addDatatype("boolseq", boolean[].class,
            BoolSeqPushInterface.BoolSeqPushConnector.class,
            BoolSeqPullInterface.BoolSeqPullConnector.class);

    }

    public static boolean supportsClass(String _dataType) {
        return m_dataTypeMap.containsKey(_dataType);
    }
    
    public static Class lookupClass(String _dataType) {
        return m_dataTypeMap.get(_dataType.toLowerCase());
    }

    public static Class lookupPushConnector(Class _dataType) {
        return m_pushConnectorMap.get(_dataType);
    }

    public static Class lookupPullConnector(Class _dataType) {
        return m_pullConnectorMap.get(_dataType);
    }

    public static void addDatatype(String _dataType, Class _dataClass,
            Class _pushRecvClass, Class _pullRecvClass) {
        m_dataTypeMap.put(_dataType.toLowerCase(), _dataClass);
        m_pushConnectorMap.put(_dataClass, _pushRecvClass);
        m_pullConnectorMap.put(_dataClass, _pullRecvClass);
    }

    // protected static class StringPushConnectionCreator implements
    // PushConnectionCreator {
    //
    // /*
    // * (non-Javadoc)
    // *
    // * @see
    // balt.core.connectors.LocalConnectionFactory.PushConnectionCreator#connect(balt.core.connectors.PushSender,
    // * balt.core.connectors.PushReceiver,
    // * java.lang.String)
    // */
    // public PushConnector connect(PushSender _send,
    // PushReceiver _recv, String _id) {
    // PushSender.StringSender send = (PushSender.StringSender) _send;
    // PushReceiver.StringPushReceiver recv =
    // (PushReceiver.StringPushReceiver) _recv;
    // StringPushConnectorImpl conn = new StringPushConnectorImpl();
    // send.setPushConnector(conn);
    // conn.registerPushReceiver(recv);
    // return conn;
    // }
    //
    // }

    // protected static class LongSeqPushConnectionCreator implements
    // PushConnectionCreator {
    //
    // /*
    // * (non-Javadoc)
    // *
    // * @see
    // balt.core.connectors.LocalConnectionFactory.PushConnectionCreator#connect(balt.core.connectors.PushSender,
    // * balt.core.connectors.PushReceiver,
    // * java.lang.String)
    // */
    // public PushConnector connect(PushSender _send,
    // PushReceiver _recv, String _id) {
    // PushSender.LongSeqSender send = (PushSender.LongSeqSender) _send;
    // PushReceiver.LongSeqPushReceiver recv =
    // (PushReceiver.LongSeqPushReceiver) _recv;
    // LongSeqPushConnectorImpl conn = new LongSeqPushConnectorImpl();
    // send.setPushConnector(conn);
    // conn.registerPushReceiver(recv);
    // return conn;
    // }
    //
    // }

    /**
     * This class defines a one-to-one pull relationship. Not fully
     * implemented/tested in whole balt. \todo Add pull
     * functionality to balt.
     * 
     * @author nah
     */
    protected static class PullConnectorImpl implements PullConnector {

        /**
         * Process to receive results of pull.
         */
        private PullReceiver m_output;

        /**
         * Default constructore. Initialises receiver to null.
         */
        public PullConnectorImpl() {
            m_output = null;
        }

        /*
         * (non-Javadoc)
         * 
         * @see balt.prototypes.connectors.FrameworkPullConnector#pull(balt.prototypes.data.FrameworkQuery)
         */
        public FrameworkLocalData<?> pull(FrameworkQuery _query)
                throws FrameworkConnectionException {
            // if (m_output != null) {
            // FrameworkLocalData<?> data = m_output
            // .receivePullQuery(_query);
            // return data;
            // }
            // else {
            // throw new FrameworkConnectionException(
            // "m_output not set. have you called setPullReceiver?");
            // }
            return null;
        }

        /*
         * (non-Javadoc)
         * 
         * @see balt.prototypes.connectors.FrameworkPullConnector#setPullReceiver(balt.prototypes.connectors.PullReceiver)
         */
        public void setPullReceiver(PullReceiver _pr) {
            m_output = _pr;
        }

        /* (non-Javadoc)
         * @see balt.core.connectors.FrameworkConnector#stopConnector()
         */
        public void stopConnector() {
            // TODO Auto-generated method stub
            
        }

    }

}
