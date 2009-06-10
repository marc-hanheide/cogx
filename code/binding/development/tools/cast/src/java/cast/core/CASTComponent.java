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
 * Basic components that are used to make up a CAAT architecture.
 */
package cast.core;

import java.util.Map;
import java.util.concurrent.Semaphore;

import Ice.Current;
import Ice.Identity;
import Ice.ObjectAdapter;
import cast.cdl.CASTTime;
import cast.cdl.COMPONENTNUMBERKEY;
import cast.cdl.DEBUGKEY;
import cast.cdl.LOGKEY;
import cast.interfaces.ComponentManagerPrx;
import cast.interfaces.TimeServerPrx;
import cast.interfaces._CASTComponentOperations;

/**
 * A basic component in the CAST framework. This class provides some basic
 * functionality useful for debugging
 * 
 * @author nah
 */
public abstract class CASTComponent implements _CASTComponentOperations,
		Runnable {

	/**
	 * The CAST time server.
	 */
	private TimeServerPrx m_timeServer;

	// determine whether debug output should be produced
	protected boolean m_bDebugOutput;

	// determine whether log output should be produced
	protected boolean m_bLogOutput;

	// access controller used to prevent asynchronous access
	protected Semaphore m_semaphore;

	protected boolean m_asleep;

	protected Object m_unlockNotification;

	private String m_startColourEscape;

	private boolean m_configureCalled;

	private boolean m_startCalled;

	private String m_componentID;

	private ComponentManagerPrx m_manager;

	private Thread m_runThread;

	// ice thingies
	private ObjectAdapter m_adapter;
	private Identity m_iceIdentity;

	/**
	 * Construct a new component with the given unique ID.
	 * 
	 * @param _id
	 *            The id used to identify this component.
	 */
	public CASTComponent() {
		// new fair (FIFO) semaphore with a single permit
		m_semaphore = new Semaphore(1, true);
		m_bLogOutput = false;
		m_bDebugOutput = false;
		m_unlockNotification = new Object();
		m_startCalled = false;
		m_configureCalled = false;
		m_startColourEscape = END_COLOUR_ESCAPE;
		m_componentID = null;
		m_manager = null;
	}

	protected CASTTime getCASTTime() {
		assert(m_timeServer != null);
		return m_timeServer.getCASTTime();
	}

	/**
	 * Print some debugging output.
	 * 
	 * @param _o
	 */
	protected void debug(Object _o) {
		String out = _o.toString();

		if (m_bDebugOutput) {
			String id = getComponentID().toString();

			// reduce overhead of string concat
			StringBuffer buff = new StringBuffer(12 + id.length()
					+ out.length());

			startColourise(buff);
			buff.append("[DEBUG \"");
			buff.append(id);
			buff.append("\": ");
			buff.append(out);
			buff.append("]");
			endColourise(buff);

			System.out.println(buff.toString());
		}
	}

	/**
	 * Use getComponentID instead.
	 * 
	 * @return
	 */
	@Deprecated
	public String getProcessIdentifier() {
		return getComponentID();
	}

	/**
	 * Gets the unique ID of this component.
	 * 
	 * @return
	 */
	public String getComponentID() {
		return m_componentID;
	}

	public String getID(Current __current) {
		return getComponentID();
	}

	/**
	 * Wait until the unlockComponent is successfully called.
	 */
	protected void waitForUnlock() {

		synchronized (m_unlockNotification) {
			try {
				m_unlockNotification.wait();
			} catch (InterruptedException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}
	}

	/**
	 * Acquire the semaphore for access to this component.
	 */
	protected void lockComponent() {

		try {
			m_semaphore.acquire();
			// debug code to find deadlocks
			// if (!m_semaphore.tryAcquire(1, TimeUnit.SECONDS)) {
			// throw new RuntimeException("Unable to access queue!");
			// }
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Print some logging output.
	 * 
	 * @param _o
	 */
	protected void log(Object _o) {
		String out = _o.toString();

		if (m_bLogOutput) {

			String id = getComponentID();

			// reduce overhead of string concat
			StringBuffer buff = new StringBuffer(10 + id.length()
					+ out.length());
			startColourise(buff);
			buff.append("[LOG \"");
			buff.append(id);
			buff.append("\": ");
			buff.append(out);
			buff.append("]");
			endColourise(buff);
			System.out.println(buff.toString());

		}

	}

	/**
	 * Print out the input in a formatted way.
	 * 
	 * @param _s
	 *            String to print.
	 */
	protected void println(Object _o) {
		String out = _o.toString();
		String id = getComponentID();

		// reduce overhead of string concat
		StringBuffer buff = new StringBuffer(6 + id.length() + out.length());

		startColourise(buff);
		buff.append("[\"");
		buff.append(id);
		buff.append("\": ");
		buff.append(out);
		buff.append("]");
		endColourise(buff);

		System.out.println(buff.toString());

	}

	private void startColourise(StringBuffer _buff) {
		_buff.append(m_startColourEscape);
	}

	private void endColourise(StringBuffer _buff) {
		_buff.append(END_COLOUR_ESCAPE);
	}

	/**
	 * Method called in separate thread to run processing component. This method
	 * is called for each component when it is started by the framework process
	 * server.
	 */
	protected void runComponent() {
	}

	/**
	 * Put the processes thread to sleep for a number of milliseconds.
	 * 
	 * @param _millis
	 *            Number of milliseconds to sleep for.
	 */
	protected void sleepComponent(long _millis) {
		try {
			m_asleep = true;
			Thread.sleep(_millis);
			m_asleep = false;
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Release the semaphore for access to this component.
	 */
	protected void unlockComponent() {
		m_semaphore.release();
		synchronized (m_unlockNotification) {
			m_unlockNotification.notifyAll();
		}
	}

	private final static String END_COLOUR_ESCAPE = "\033[0m";

	protected void startInternal() {
		debug("CASTComponent.startInternal()");
		m_startCalled = true;
	}

	public final boolean isRunning() {
		return m_startCalled;
	}

	protected void start() {

	}

	protected void configure(Map<String, String> _config) {
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * balt.core.processes.FrameworkComponent#configure(java.util.Properties)
	 */
	protected void configureInternal(Map<String, String> __config) {
		// System.out.println("CAATComponent.configure()");

		m_configureCalled = true;

		String logString = __config.get(LOGKEY.value);

		if (logString != null) {
			m_bLogOutput = Boolean.parseBoolean(logString);
			// println("setting log to: " + m_bLogOutput);
		}

		String debugString = __config.get(DEBUGKEY.value);

		if (debugString != null) {
			m_bDebugOutput = Boolean.parseBoolean(debugString);
			// println("setting log to: " + m_bLogOutput);
		}

		String numberString = __config.get(COMPONENTNUMBERKEY.value);
		assert (numberString != null);
		int myNumber = Integer.parseInt(numberString);
		int printNumber = (myNumber % 7);
		int bold = (myNumber / 7) % 2;
		if (printNumber == 0) {
			// 0 = do nothing
			m_startColourEscape = "\033[0m";
		} else {
			// otherwise use the connected colour
			m_startColourEscape = "\033[3" + printNumber;
			if (bold != 0) {
				m_startColourEscape += ";1m";
			} else {
				m_startColourEscape += "m";
			}
		}
		// println("MEMEMEMEMEMEMEME");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see balt.core.processes.FrameworkComponent#stop()
	 */
	protected void stopInternal() {
		try {
			m_runThread.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		synchronized (m_unlockNotification) {
			m_unlockNotification.notifyAll();
		}
	}

	protected void stop() {
	}

	public void beat(Current __current) {
	}

	public void configure(Map<String, String> _config, Current __current) {
		configureInternal(_config);
		configure(_config);
	}

	public void run(Current __current) {
		m_runThread = new Thread(this);
		m_runThread.start();
	}

	public void setComponentManager(ComponentManagerPrx _man, Current __current) {
		m_manager = _man;
	}

	public void setID(String _id, Current __current) {
		assert (m_componentID == null);
		m_componentID = _id;
	}

	public void start(Current __current) {
		startInternal();
		start();
	}

	public void stop(Current __current) {
		m_startCalled = false;
		stop();
		stopInternal();
		// remove self from adapter
		// getObjectAdapter().remove(getIceIdentity());
	}

	/**
	 * Called when the thread starts at runtime
	 */
	public void run() {
		assert (m_startCalled);
		assert (m_configureCalled);
		try {
			runComponent();
		} catch (RuntimeException e) {
			println("Caught RuntimeException and rethrowing: " + e.getMessage());
			throw e;
		}
	}

	/**
	 * @param adapter
	 *            the adapter to set
	 */
	public void setObjectAdapter(ObjectAdapter adapter) {
		m_adapter = adapter;
	}

	/**
	 * @return the adapter
	 */
	public ObjectAdapter getObjectAdapter() {
		assert (m_adapter != null);
		return m_adapter;
	}

	/**
	 * @param iceIdentity
	 *            the iceIdentity to set
	 */
	public void setIceIdentity(Identity iceIdentity) {
		m_iceIdentity = iceIdentity;
	}

	/**
	 * @return the iceIdentity
	 */
	public Identity getIceIdentity() {
		assert (m_iceIdentity != null);
		return m_iceIdentity;
	}

	public void destroy(Current __current) {
		__current.adapter.remove(__current.id);
	}
	
	public void setTimeServer(TimeServerPrx _ts, Current __current) {
		m_timeServer = _ts;
	}
}
