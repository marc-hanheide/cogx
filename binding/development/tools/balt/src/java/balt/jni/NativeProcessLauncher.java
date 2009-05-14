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
package balt.jni;

import java.util.Properties;

import balt.corba.autogen.FrameworkBasics.BALTTime;
import balt.management.CPPProcessDescription;

/**
 * Lauches C++ process via the JNI
 * 
 * @author nah
 */
public class NativeProcessLauncher {

	static {
	    loadLibrary("FrameworkJNI");
	}

	private static void loadLibrary(String _libName) {
		try {
			System.loadLibrary(_libName);

		} catch (UnsatisfiedLinkError e) {

			System.err.println("Cannot load or link " + _libName);
			System.err.println("Exception msg: " + e.getMessage());
			System.err.println("Library path: "
					+ System.getProperty("java.library.path"));

			e.printStackTrace();

			System.exit(1);
		}
	}

	@Override
	protected void finalize() throws Throwable {
		super.finalize();
		System.out.println("NativeProcessLauncher.finalize()");
	}

	private static native void cleanupNativeInterface();

	private static native void connectRemotePullConnectionReceiver(
			String _procName, String _remoteID, String _dataType,
			String _connectionID);

	private static native void connectRemotePullConnectionSender(
			String _procName, String _remoteID, String _dataType,
			String _connectionID);

	private static native void connectRemotePushConnectionReceiver(
			String _procName, String _remoteID, String _dataType,
			String _connectionID);

	private static native void connectRemotePushConnectionSender(
			String _procName, String _remoteID, String _dataType,
			String _connectionID);

	private static native void createNativeProcess(String _class,
			String _procName, String[] _configKeys, String[] _configValues);

	private static native void createNativePullConnection(String[] _senderName,
			String _receiverName, String _dataType, String _connectionID);

	private static native void createNativePushConnection(String[] _senderName,
			String _receiverName[], String _dataType, String _connectionID);

	private static native synchronized BALTTime getNativeBALTTime();

	// private static native synchronized BALTTime nativeTimeDiff(
	// BALTTime _start, BALTTime _end);

	private static native void init(String _namingHost, String _namingPort);

	private static native void resetNativeClock();

	private static native void startNativeProcess(String _procName);

	private static native void runNativeProcess(String _procName);

	private static native void stopNativeProcess(String _procName);

	private static String[] toNameArray(CPPProcessDescription[] _descs) {
		String[] names = new String[_descs.length];
		for (int i = 0; i < _descs.length; i++) {
			names[i] = _descs[i].getProcessName();
		}
		return names;
	}

	public static void cleanupInterface() {
		cleanupNativeInterface();
	}

	/**
	 * @param _procDescA
	 * @param _remoteID
	 */
	public static void connectRemotePullConnectionReceiver(
			CPPProcessDescription _procDescA, String _remoteID,
			String _dataType, String _connectionID) {
		connectRemotePullConnectionReceiver(_procDescA.getProcessName(),
				_remoteID, _dataType, _connectionID);
	}

	/**
	 * @param _procDescA
	 * @param _remoteID
	 */
	public static void connectRemotePullConnectionSender(
			CPPProcessDescription _procDescA, String _remoteID,
			String _dataType, String _connectionID) {
		connectRemotePullConnectionSender(_procDescA.getProcessName(),
				_remoteID, _dataType, _connectionID);
	}

	/**
	 * @param _procDescA
	 * @param _remoteID
	 */
	public static void connectRemotePushConnectionReceiver(
			CPPProcessDescription _procDescA, String _remoteID,
			String _dataType, String _connectionID) {
		connectRemotePushConnectionReceiver(_procDescA.getProcessName(),
				_remoteID, _dataType, _connectionID);
	}

	/**
	 * @param _procDescA
	 * @param _remoteID
	 */
	public static void connectRemotePushConnectionSender(
			CPPProcessDescription _procDescA, String _remoteID,
			String _dataType, String _connectionID) {
		connectRemotePushConnectionSender(_procDescA.getProcessName(),
				_remoteID, _dataType, _connectionID);
	}

	public static void createLocalPullConnection(
			CPPProcessDescription[] _sender, CPPProcessDescription _receiver,
			String _dataType, String _connectionID) {
		createNativePullConnection(toNameArray(_sender), _receiver
				.getProcessName(), _dataType, _connectionID);
	}

	public static void createLocalPushConnection(
			CPPProcessDescription[] _sender, CPPProcessDescription[] _receiver,
			String _dataType, String _connectionID) {

		createNativePushConnection(toNameArray(_sender),
				toNameArray(_receiver), _dataType, _connectionID);

	}

	public static BALTTime getBALTTime() {
		return getNativeBALTTime();
	}

	// public static BALTTime timeDiff(BALTTime _start, BALTTime _end) {
	// return nativeTimeDiff(_start, _end);
	// }

	public static BALTTime timeDiff(BALTTime _start, BALTTime _end) {
		BALTTime startTime = new BALTTime(_start.m_s, _start.m_us);

		/* start the carry for the later subtraction by updating y. */
		if (_end.m_us < startTime.m_us) {
			int nsec = (startTime.m_us - _end.m_us) / 1000000 + 1;
			startTime.m_us -= 1000000 * nsec;
			startTime.m_s += nsec;
		}
		if (_end.m_us - startTime.m_us > 1000000) {
			int nsec = (_end.m_us - startTime.m_us) / 1000000;
			startTime.m_us += 1000000 * nsec;
			startTime.m_s -= nsec;
		}

		/*
		 * Compute the time remaining to wait. m_us is certainly positive.
		 */
		return new BALTTime(_end.m_s - startTime.m_s, _end.m_us
				- startTime.m_us);
	}

	public static double toMillis(BALTTime _time) {
		return (_time.m_s * 1000d) + (_time.m_us / 1000d);
	}

	public static double toSeconds(BALTTime _time) {
		return _time.m_s + (_time.m_us / 1000000d);
	}

	public static String toString(BALTTime _time) {
		return Double.toString(toSeconds(_time));
	}

	/**
	 * @param _args
	 * @param _props
	 */
	public static void init(String[] _args, Properties _props) {
		init((String) _props.get("org.omg.CORBA.ORBInitialHost"),
				(String) _props.get("org.omg.CORBA.ORBInitialPort"));
	}

	public static void launchProcess(CPPProcessDescription _desc) {

		// test hack
//	    loadLibrary(_desc.getProcessClass());

		// to lauch in c we need a string for the name, and the config
		// details

		Properties config = _desc.getProcessConfiguration();

		String[] configKeys = (String[]) config.keySet().toArray(
				new String[config.size()]);
		String[] configValues = new String[configKeys.length];

		for (int i = 0; i < configKeys.length; i++) {
			configValues[i] = config.getProperty(configKeys[i]);
		}

		createNativeProcess(_desc.getProcessClass(), _desc.getProcessName(),
				configKeys, configValues);
	}

	/**
	 * 
	 */
	public static void resetClock() {
		resetNativeClock();
	}

	/**
	 * @param _desc
	 */
	public static void startProcess(CPPProcessDescription _desc) {
		startNativeProcess(_desc.getProcessName());
	}

	/**
	 * @param _desc
	 */
	public static void runProcess(CPPProcessDescription _desc) {
		runNativeProcess(_desc.getProcessName());
	}

	/**
	 * @param _desc
	 */
	public static void stopProcess(CPPProcessDescription _desc) {
		stopNativeProcess(_desc.getProcessName());
	}

	public static native int getProcessID();
	
}
