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
package cast.core.components;

import java.util.Properties;
import java.util.concurrent.Semaphore;

import cast.cdl.COMPONENT_NUMBER_KEY;
import cast.cdl.DEBUG_KEY;
import cast.cdl.LOG_KEY;

/**
 * A basic component in the CAAT framework. This class provides some basic
 * functionality useful for debugging, and also overrides some of the abstract
 * methodsd on FrameworkProcess so that all processes in C++ and Java look as
 * similar as possible.
 * 
 * @see framework.core.processes.FrameworkProcess
 * @author nah
 */
public abstract class CASTComponent extends InspectableComponent {

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

	/**
	 * Construct a new component with the given unique ID.
	 * 
	 * @param _id
	 *            The id used to identify this component.
	 */
	public CASTComponent(String _id) {
		super(_id);
		// new fair (FIFO) semaphore with a single permit
		m_semaphore = new Semaphore(1, true);
		m_bLogOutput = false;
		m_bDebugOutput = false;
		m_unlockNotification = new Object();
		m_startCalled = false;
		m_configureCalled = false;
		m_startColourEscape = END_COLOUR_ESCAPE;
	}

	/**
	 * Print some debugging output.
	 * 
	 * @param _o
	 */
	protected void debug(Object _o) {
		String out = _o.toString();

		if (m_bDebugOutput) {
			String id = getProcessIdentifier().toString();

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
	 * Wait until the unlockProcess is successfully called.
	 */
	protected void waitForUnlock() {

		synchronized (m_unlockNotification) {
			try {
				m_unlockNotification.wait();
			}
			catch (InterruptedException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}
	}

	/**
	 * Acquire the semaphore for access to this component.
	 */
	protected void lockProcess() {

		try {
			m_semaphore.acquire();
			// debug code to find deadlocks
			// if (!m_semaphore.tryAcquire(1, TimeUnit.SECONDS)) {
			// throw new RuntimeException("Unable to access queue!");
			// }
		}
		catch (InterruptedException e) {
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

			String id = getProcessIdentifier().toString();

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
		String id = getProcessIdentifier().toString();

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
	protected abstract void runComponent();

	/**
	 * Put the processes thread to sleep for a number of milliseconds.
	 * 
	 * @param _millis
	 *            Number of milliseconds to sleep for.
	 */
	protected void sleepProcess(long _millis) {
		try {
			m_asleep = true;
			Thread.sleep(_millis);
			m_asleep = false;
		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Release the semaphore for access to this component.
	 */
	protected void unlockProcess() {
		m_semaphore.release();
		synchronized (m_unlockNotification) {
			m_unlockNotification.notifyAll();
		}
	}

	private final static String END_COLOUR_ESCAPE = "\033[0m";

	@Override
	public void start() {
		super.start();
		m_startCalled = true;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see balt.core.processes.FrameworkProcess#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config) {
		// System.out.println("CAATComponent.configure()");

		m_configureCalled = true;

		String logString = _config.getProperty(LOG_KEY.value);

		if (logString != null) {
			m_bLogOutput = Boolean.parseBoolean(logString);
			// println("setting log to: " + m_bLogOutput);
		}

		String debugString = _config.getProperty(DEBUG_KEY.value);

		if (debugString != null) {
			m_bDebugOutput = Boolean.parseBoolean(debugString);
			// println("setting log to: " + m_bLogOutput);
		}

		String numberString = _config.getProperty(COMPONENT_NUMBER_KEY.value);
		assert (numberString != null);
		int myNumber = Integer.parseInt(numberString);
		int printNumber = (myNumber % 7);
		int bold = (myNumber / 7) % 2;
		if (printNumber == 0) {
			// 0 = do nothing
			m_startColourEscape = "\033[0m";
		}
		else {
			// otherwise use the connected colour
			m_startColourEscape = "\033[3" + printNumber;
			if (bold != 0) {
				m_startColourEscape += ";1m";
			}
			else {
				m_startColourEscape += "m";
			}
		}
		// println("MEMEMEMEMEMEMEME");
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see balt.core.processes.FrameworkProcess#stop()
	 */
	@Override
	public void stop() {
		super.stop();
		synchronized (m_unlockNotification) {
			m_unlockNotification.notifyAll();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see framework.core.processes.FrameworkProcess#run()
	 */
	@Override
	public void run() {
		assert(m_startCalled);
		assert(m_configureCalled);
		runComponent();
	}

}
