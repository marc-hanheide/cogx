/*
 * BALT - The Boxes and Lines Toolkit for component communication.
 * Copyright (C) 2006-2007 Nick Hawes This library is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * Lesser General Public License as published by the Free Software
 * Foundation; either version 2.1 of the License, or (at your option)
 * any later version. This library is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU Lesser General Public License for more details. You should
 * have received a copy of the GNU Lesser General Public License along
 * with this library; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * 
 */
package balt.corba.connectors.push;

import org.omg.CORBA.SystemException;

import balt.corba.autogen.RemoteConnectors.RemotePushConnector;
import balt.corba.data.RemoteDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.core.connectors.push.nonprimitive.impl.*;

/**
 * Worker class for remote senders. Now updated to push the translation down
 * into a separate thread. This means that the sending process doesn't have to
 * wait while translation is done when sending.
 * 
 * @author nah
 */
public class GenericRemotePushSender<D> {

	private class TranslatingPushConnectorRunnable
			extends
				GenericPushConnectorRunnable<RemotePushConnector, DataPair<D>> {

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.core.connectors.push.nonprimitive.impl.GenericPushConnectorRunnable#sendData(java.lang.Object,
		 *      java.lang.Object)
		 */
		@Override
		protected void sendData(RemotePushConnector _receiver, DataPair<D> _send) {
			try {
				_receiver.push(_send.getSrc(), m_translator.translate(_send
						.getData()));
			}
			catch (FrameworkDataTranslatorException e) {
				e.printStackTrace();
				System.exit(1);
			}
			catch (SystemException e) {
				System.out
						.println("TranslatingPushConnectorRunnable.sendData(): " + _send.getSrc() + " sent " + _send.getData().getClass());
				e.printStackTrace();
				System.exit(0);
			}
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.core.connectors.push.nonprimitive.impl.GenericPushConnectorRunnable#flush()
		 */
		@Override
		public void flush() {
			// flush all data to remote connectors
			super.flush();
			// then flush connectors
			for (RemotePushConnector conn : m_receivers) {
				conn.flush();
			}
		}

	}

	private TranslatingPushConnectorRunnable m_senderRunnable;

	// TranslatingP

	private FrameworkDataTranslator<D> m_translator;

	private Thread m_senderThread;

	/**
	 * @param _data
	 */
	@SuppressWarnings("unchecked")
	public GenericRemotePushSender(Class<D> _dataClass) {
		m_translator = (FrameworkDataTranslator<D>) RemoteDataTranslator
				.getTranslator(_dataClass);

		if (m_translator == null) {
			throw new RuntimeException("Unable to get translator for "
					+ _dataClass);
		}

		m_senderRunnable = new TranslatingPushConnectorRunnable();
		m_senderThread = new Thread(m_senderRunnable);
	}

	public void flush() {
		// System.out
		// .println("GenericRemotePushSender.flush()");
		m_senderRunnable.flush();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see balt.core.connectors.PushConnectorOut.StringPushConnectorOut#push(java.lang.String,
	 *      java.lang.String)
	 */
	@SuppressWarnings("unchecked")
	public void push(String _src, D _data) {
		// System.out.println("GenericRemotePushSender.push()");
		m_senderRunnable.queue(new DataPair(_src, _data));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see balt.core.connectors.PushConnectorRegister.StringPushConnectorRegister#registerPushReceiver(balt.core.connectors.PushReceiver.StringPushReceiver)
	 */
	// public void registerPushReceiver(RemotePushConnector _pr) {
	// if (!m_senderRunnable.isRunning()) {
	// m_senderRunnable.start();
	// m_senderThread.start();
	// }
	//
	// m_senderRunnable.registerPushReceiver(_pr);
	// }
	/*
	 * (non-Javadoc)
	 * 
	 * @see balt.prototypes.sandbox.corba.RemoteConnectors.RemotePushSenderOperations#setPushConnector(balt.prototypes.sandbox.corba.RemoteConnectors.RemotePushConnector)
	 */
	public void setPushConnector(RemotePushConnector _out) {
		if (!m_senderRunnable.isRunning()) {
			m_senderRunnable.start();
			m_senderThread.start();
		}

		m_senderRunnable.registerPushReceiver(_out);

	}

}
