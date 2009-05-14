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
package balt.corba.impl;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Properties;

import org.omg.CORBA.Any;
import org.omg.CORBA.BAD_PARAM;
import org.omg.CORBA.LocalObject;
import org.omg.CORBA.ORB;
import org.omg.CORBA.Policy;
import org.omg.CORBA.SystemException;
import org.omg.CosNaming.*;
import org.omg.CosNaming.NamingContextPackage.CannotProceed;
import org.omg.CosNaming.NamingContextPackage.InvalidName;
import org.omg.CosNaming.NamingContextPackage.NotFound;
import org.omg.PortableServer.*;
import org.omg.PortableServer.POAPackage.ServantNotActive;
import org.omg.PortableServer.POAPackage.WrongPolicy;

import balt.corba.autogen.RemoteConnectors.*;
import balt.corba.connectors.pull.GenericRemotePullSender;
import balt.corba.connectors.pull.primitive.*;
import balt.corba.connectors.push.GenericRemotePushSender;
import balt.corba.connectors.push.primitive.*;
import balt.corba.data.RemoteDataTranslator;
import balt.corba.data.translation.FrameworkDataTranslatorException;
import balt.core.autocode.ConnectorGenerator;
import balt.core.connectors.*;
import balt.core.connectors.pull.*;
import balt.core.connectors.pull.primitive.interfaces.*;
import balt.core.connectors.push.*;
import balt.core.connectors.push.primitive.interfaces.*;
import balt.core.data.FrameworkQuery;
import balt.management.*;

/**
 * Main class for handling connections between remote and cross-language
 * components. This class creates all the CORBA connection servers that will be
 * used on the current machine. Try to keep as much CORBA code as possible in
 * here so it can be "easily" replaced.
 * 
 * @author nah
 */
public class RemoteConnectionManager {

	public static class RemoteConnectionManagerAdapterActivator
			extends
				LocalObject implements AdapterActivator {

		/**
		 * 
		 */
		private static final long serialVersionUID = 3364265298116815615L;

		public boolean unknown_adapter(POA parent, String name) {
			// System.out.println("unknown_adapter() invoked for POA - "
			// + name);
			try {
				// create the POA with appropriate policies
				Policy[] policy = new Policy[3];
				policy[0] = parent
						.create_lifespan_policy(LifespanPolicyValue.TRANSIENT);
				policy[1] = parent
						.create_id_assignment_policy(IdAssignmentPolicyValue.SYSTEM_ID);
				policy[2] = parent
						.create_implicit_activation_policy(ImplicitActivationPolicyValue.NO_IMPLICIT_ACTIVATION);

				POA child = parent.create_POA(name, null, policy);

				if (name.startsWith(PUSH_CONNECTOR_PREFIX)) {

					// Associate the servant with the new POA
					RemotePushConnectorImpl rpci = new RemotePushConnectorImpl();

					child.activate_object_with_id(name.getBytes(), rpci);

					// store the connector impl for debugging
					m_pushConnectors.add(rpci);
				}
				else if (name.startsWith(PULL_CONNECTOR_PREFIX)) {
					// Associate the servant with the new POA
					RemoteConnectionManager.RemotePullConnectorImpl rpci = new RemoteConnectionManager.RemotePullConnectorImpl();
					child.activate_object_with_id(name.getBytes(), rpci);

					// store the connector impl for debugging
					m_pullConnectors.add(rpci);
				}

				// activate the new POA
				child.the_POAManager().activate();

				return true;
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			return false;
		}
	}

	public static class RemotePullConnectorImpl extends RemotePullConnectorPOA {

		private RemotePullReceiver m_out;

		/**
		 * 
		 */
		public RemotePullConnectorImpl() {
			m_out = null;
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.corba.autogen.RemoteConnectors.RemotePullConnectorOperations#pull(java.lang.String,
		 *      java.lang.String)
		 */
		public Any pull(String _src, String _query) {
			if (m_out != null) {
				return m_out.receivePullQuery(_src, _query);
			}
			else {
				System.err
						.println("RemotePullConnectorImpl has no pull target!");
				System.err.println("Have you called registerPullReceiver?");
				return null;
			}
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.corba.autogen.RemoteConnectors.RemotePullConnectorOperations#registerPullReceiver(balt.corba.autogen.RemoteConnectors.RemotePullReceiver)
		 */
		public void registerPullReceiver(RemotePullReceiver _pr) {
			m_out = _pr;
		}

	}

	public static class RemotePullReceiverImpl<R, D>
			extends
				RemotePullReceiverPOA implements PullConnectorRegister {

		private abstract static class RemotePullReceiverHelper {

			public abstract Any receivePullQuery(String _src, String _query);
		}

		private R m_out;
		private Method m_pullMethod;
		private RemotePullReceiverHelper helper;

		/**
		 * 
		 */
		public RemotePullReceiverImpl(Class<R> _recvClass, Class<D> _dataClass) {

			if (_dataClass.isPrimitive()) {
				if (_dataClass == int.class) {
					helper = new RemotePullReceiverHelper() {

						@Override
						public Any receivePullQuery(String _src, String _query) {
							FrameworkQuery query = new FrameworkQuery(_src,
									_query);
							try {
								int data = ((IntPullInterface.IntPullReceiver) m_out)
										.receivePullQueryInt(query);
								return RemoteDataTranslator
										.translateToAny(data);
							}
							catch (FrameworkDataTranslatorException e) {
								e.printStackTrace();
								System.exit(1);
								return null;
							}
						}
					};
				}
				else if (_dataClass == long.class) {
					helper = new RemotePullReceiverHelper() {

						@Override
						public Any receivePullQuery(String _src, String _query) {
							FrameworkQuery query = new FrameworkQuery(_src,
									_query);
							try {
								long data = ((LongPullInterface.LongPullReceiver) m_out)
										.receivePullQueryLong(query);
								return RemoteDataTranslator
										.translateToAny(data);
							}
							catch (FrameworkDataTranslatorException e) {
								e.printStackTrace();
								System.exit(1);
								return null;
							}
						}
					};
				}
				else if (_dataClass == short.class) {
					helper = new RemotePullReceiverHelper() {

						@Override
						public Any receivePullQuery(String _src, String _query) {
							FrameworkQuery query = new FrameworkQuery(_src,
									_query);
							try {
								short data = ((ShortPullInterface.ShortPullReceiver) m_out)
										.receivePullQueryShort(query);
								return RemoteDataTranslator
										.translateToAny(data);
							}
							catch (FrameworkDataTranslatorException e) {
								e.printStackTrace();
								System.exit(1);
								return null;
							}
						}
					};
				}
				else if (_dataClass == double.class) {
					helper = new RemotePullReceiverHelper() {

						@Override
						public Any receivePullQuery(String _src, String _query) {
							FrameworkQuery query = new FrameworkQuery(_src,
									_query);
							try {
								double data = ((DoublePullInterface.DoublePullReceiver) m_out)
										.receivePullQueryDouble(query);
								return RemoteDataTranslator
										.translateToAny(data);
							}
							catch (FrameworkDataTranslatorException e) {
								e.printStackTrace();
								System.exit(1);
								return null;
							}
						}
					};
				}
				else if (_dataClass == float.class) {
					helper = new RemotePullReceiverHelper() {

						@Override
						public Any receivePullQuery(String _src, String _query) {
							FrameworkQuery query = new FrameworkQuery(_src,
									_query);
							try {
								float data = ((FloatPullInterface.FloatPullReceiver) m_out)
										.receivePullQueryFloat(query);
								return RemoteDataTranslator
										.translateToAny(data);
							}
							catch (FrameworkDataTranslatorException e) {
								e.printStackTrace();
								System.exit(1);
								return null;
							}
						}
					};
				}
				else if (_dataClass == char.class) {
					helper = new RemotePullReceiverHelper() {

						@Override
						public Any receivePullQuery(String _src, String _query) {
							FrameworkQuery query = new FrameworkQuery(_src,
									_query);
							try {
								char data = ((CharPullInterface.CharPullReceiver) m_out)
										.receivePullQueryChar(query);
								return RemoteDataTranslator
										.translateToAny(data);
							}
							catch (FrameworkDataTranslatorException e) {
								e.printStackTrace();
								System.exit(1);
								return null;
							}
						}
					};
				}

				else if (_dataClass == byte.class) {
					helper = new RemotePullReceiverHelper() {

						@Override
						public Any receivePullQuery(String _src, String _query) {
							FrameworkQuery query = new FrameworkQuery(_src,
									_query);
							try {
								byte data = ((BytePullInterface.BytePullReceiver) m_out)
										.receivePullQueryByte(query);
								return RemoteDataTranslator
										.translateToAny(data);
							}
							catch (FrameworkDataTranslatorException e) {
								e.printStackTrace();
								System.exit(1);
								return null;
							}
						}
					};
				}

				else if (_dataClass == boolean.class) {
					helper = new RemotePullReceiverHelper() {

						@Override
						public Any receivePullQuery(String _src, String _query) {
							FrameworkQuery query = new FrameworkQuery(_src,
									_query);
							try {
								boolean data = ((BoolPullInterface.BoolPullReceiver) m_out)
										.receivePullQueryBool(query);
								return RemoteDataTranslator
										.translateToAny(data);
							}
							catch (FrameworkDataTranslatorException e) {
								e.printStackTrace();
								System.exit(1);
								return null;
							}
						}
					};
				}

				else {
					throw new RuntimeException(
							"unimplemented primitive pull reciever: "
									+ _dataClass);
				}
			}
			else {

				m_pullMethod = FrameworkUtils.findReturningMethodPrefix(
						_recvClass, _dataClass,
						PullReceiver.PULL_RECEIVER_METHOD, 1);

				assert m_pullMethod != null : "Unable to find suitable "
						+ PullReceiver.PULL_RECEIVER_METHOD + " in "
						+ _recvClass;

				helper = new RemotePullReceiverHelper() {

					@SuppressWarnings("unchecked")
					@Override
					public Any receivePullQuery(String _src, String _query) {
						FrameworkQuery query = new FrameworkQuery(_src, _query);
						try {
							D data = (D) m_pullMethod.invoke(m_out,
									new Object[]{query});
							return RemoteDataTranslator.translateToAny(data);
						}
						catch (IllegalArgumentException e) {
							e.printStackTrace();
							System.exit(1);
						}
						catch (IllegalAccessException e) {
							e.printStackTrace();
							System.exit(1);
						}
						catch (InvocationTargetException e) {
							e.printStackTrace();
							System.exit(1);
						}
						catch (FrameworkDataTranslatorException e) {
							e.printStackTrace();
							System.exit(1);
						}
						// never happens
						return null;
					}
				};
			}

		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.corba.autogen.RemoteConnectors.RemotePullReceiverOperations#receivePullQuery(java.lang.String,
		 *      java.lang.String)
		 */
		public Any receivePullQuery(String _src, String _query) {
			if (m_out != null) {
				try {
					return helper.receivePullQuery(_src, _query);
				}
				catch (SystemException e) {
					System.err
							.println("RemotePullReceiverImpl.receivePullQuery(): " + _src + " send query " + _query);
					e.printStackTrace();
					System.exit(0);
				}
			}
			else {
				System.err
						.println("m_out not set. have you called setPullReceiver?");
			}
			return null;
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.core.connectors.PullConnectorRegister#setPullReceiver(balt.core.connectors.PullReceiver)
		 */
		public void setPullReceiver(R _pr) {
			m_out = _pr;
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.core.connectors.FrameworkConnector#stopConnector()
		 */
		public void stopConnector() {
			// TODO Auto-generated method stub

		}
	}

	public static class RemotePushReceiverImpl<R extends PushReceiver, D>
			extends
				RemotePushReceiverPOA implements PushConnectorRegister {

		private abstract static class RemotePushReceiverHelper {

			public abstract void receivePushData(String _src, Any _data);
		}

		private R m_out;
		private Method m_pushMethod;
		private RemotePushReceiverHelper helper;
		private Class<D> m_dataClass;

		/**
		 * 
		 */
		public RemotePushReceiverImpl(Class<R> _recvClass, Class<D> _dataClass) {
			// System.out.println("receiver class: " + _recvClass);
			m_dataClass = _dataClass;

			m_pushMethod = FrameworkUtils.findAssignableMethod(_recvClass,
					_dataClass, PushReceiver.PUSH_RECEIVER_METHOD, 2, 1);

			assert m_pushMethod != null : "Unable to find suitable "
					+ PushReceiver.PUSH_RECEIVER_METHOD + " in " + _recvClass;

			if (_dataClass.isPrimitive()) {
				if (_dataClass == long.class) {
					helper = new RemotePushReceiverHelper() {

						@Override
						public void receivePushData(String _src, Any _data) {
							try {
								long data = RemoteDataTranslator
										.translateLongFromAny(_data);
								((LongPushInterface.LongPushReceiver) m_out)
										.receivePushData(_src, data);
							}
							catch (FrameworkDataTranslatorException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
								System.exit(1);
							}
						}
					};
				}
				else if (_dataClass == int.class) {
					helper = new RemotePushReceiverHelper() {

						@Override
						public void receivePushData(String _src, Any _data) {
							try {
								int data = RemoteDataTranslator
										.translateIntFromAny(_data);
								((IntPushInterface.IntPushReceiver) m_out)
										.receivePushData(_src, data);
							}
							catch (FrameworkDataTranslatorException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
								System.exit(1);
							}
						}
					};
				}
				else if (_dataClass == short.class) {
					helper = new RemotePushReceiverHelper() {

						@Override
						public void receivePushData(String _src, Any _data) {
							try {
								short data = RemoteDataTranslator
										.translateShortFromAny(_data);
								((ShortPushInterface.ShortPushReceiver) m_out)
										.receivePushData(_src, data);
							}
							catch (FrameworkDataTranslatorException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
								System.exit(1);
							}
						}
					};
				}
				else if (_dataClass == float.class) {
					helper = new RemotePushReceiverHelper() {

						@Override
						public void receivePushData(String _src, Any _data) {
							try {
								float data = RemoteDataTranslator
										.translateFloatFromAny(_data);
								((FloatPushInterface.FloatPushReceiver) m_out)
										.receivePushData(_src, data);
							}
							catch (FrameworkDataTranslatorException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
								System.exit(1);
							}
						}
					};
				}
				else if (_dataClass == double.class) {
					helper = new RemotePushReceiverHelper() {

						@Override
						public void receivePushData(String _src, Any _data) {
							try {
								double data = RemoteDataTranslator
										.translateDoubleFromAny(_data);
								((DoublePushInterface.DoublePushReceiver) m_out)
										.receivePushData(_src, data);
							}
							catch (FrameworkDataTranslatorException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
								System.exit(1);
							}
						}
					};
				}
				else if (_dataClass == byte.class) {
					helper = new RemotePushReceiverHelper() {

						@Override
						public void receivePushData(String _src, Any _data) {
							try {
								byte data = RemoteDataTranslator
										.translateByteFromAny(_data);
								((BytePushInterface.BytePushReceiver) m_out)
										.receivePushData(_src, data);
							}
							catch (FrameworkDataTranslatorException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
								System.exit(1);
							}
						}
					};
				}
				else if (_dataClass == boolean.class) {
					helper = new RemotePushReceiverHelper() {

						@Override
						public void receivePushData(String _src, Any _data) {
							try {
								boolean data = RemoteDataTranslator
										.translateBoolFromAny(_data);
								((BoolPushInterface.BoolPushReceiver) m_out)
										.receivePushData(_src, data);
							}
							catch (FrameworkDataTranslatorException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
								System.exit(1);
							}
						}
					};
				}
				else if (_dataClass == char.class) {
					helper = new RemotePushReceiverHelper() {

						@Override
						public void receivePushData(String _src, Any _data) {
							try {
								char data = RemoteDataTranslator
										.translateCharFromAny(_data);
								((CharPushInterface.CharPushReceiver) m_out)
										.receivePushData(_src, data);
							}
							catch (FrameworkDataTranslatorException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
								System.exit(1);
							}
						}
					};
				}
				else {
					throw new RuntimeException(
							"unimplemented primitive push reciever: "
									+ _dataClass);
				}

			}
			else {
				// generic object translation and receiver
				helper = new RemotePushReceiverHelper() {

					@Override
					public void receivePushData(String _src, Any _data) {
						try {
							// System.err.println(_data.type().id());
							//                            
							// System.err.println(m_dataClass);

							D data = RemoteDataTranslator.translateFromAny(
									_data, m_dataClass);
							m_pushMethod.invoke(m_out, _src, data);
						}
						catch (IllegalArgumentException e) {
							e.printStackTrace();
							System.exit(1);
						}
						catch (IllegalAccessException e) {
							e.printStackTrace();
							System.exit(1);
						}
						catch (InvocationTargetException e) {
							e.printStackTrace();
							System.exit(1);
						}
						catch (FrameworkDataTranslatorException e) {
							e.printStackTrace();
							System.exit(1);
						}
						// catch (BadKind e) {
						// e.printStackTrace();
						// System.exit(1);
						// }
					}

				};
			}

		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.corba.autogen.RemoteConnectors.RemotePushReceiverOperations#receivePushData(java.lang.String,
		 *      org.omg.CORBA.Any)
		 */
		public void receivePushData(String _src, Any _data) {
			helper.receivePushData(_src, _data);
		}

		/*
		 * (non-Javadoc)
		 * 
		 * @see balt.prototypes.connectors.PushConnector#registerPushReceiver(balt.prototypes.connectors.PushReceiver)
		 */
		public void registerPushReceiver(R _pr) {
			// System.out.println("RemotePushReceiverImpl.registerPushReceiver()");
			// register the local receiver
			m_out = _pr;
		}

	}

	private static Any m_nullAny;

	private static ORB m_orb;
	private static ArrayList<RemotePullConnectorImpl> m_pullConnectors;

	// better store these, just in case
	private static ArrayList<RemotePushConnectorImpl> m_pushConnectors;

	private static String m_namingHost = null;

	private static String m_namingPort = null;

	private static final String PULL_CONNECTOR_PREFIX = "RemotePullConnector";
	private static final String PUSH_CONNECTOR_PREFIX = "RemotePushConnector";

	static {
		m_pushConnectors = new ArrayList<RemotePushConnectorImpl>();
		m_pullConnectors = new ArrayList<RemotePullConnectorImpl>();
	}

	/**
	 * @param _dataType
	 * @param _recvClass
	 * @param _connectorClass
	 * @return
	 * @throws ConnectionCreationException
	 */
	private static <D, C> Object generateRemotePullSender(Class<D> _dataClass,
			Class<C> _connectorClass) throws ConnectionCreationException {

		if (_dataClass.isPrimitive()) {
			if (_dataClass.equals(int.class)) {
				return new IntRemotePullSenderImpl();
			}
			else if (_dataClass.equals(long.class)) {
				return new LongRemotePullSenderImpl();
			}
			else if (_dataClass.equals(short.class)) {
				return new ShortRemotePullSenderImpl();
			}
			else if (_dataClass.equals(double.class)) {
				return new DoubleRemotePullSenderImpl();
			}
			else if (_dataClass.equals(float.class)) {
				return new FloatRemotePullSenderImpl();
			}
			else if (_dataClass.equals(char.class)) {
				return new CharRemotePullSenderImpl();
			}
			else if (_dataClass.equals(byte.class)) {
				return new ByteRemotePullSenderImpl();
			}
			else if (_dataClass.equals(boolean.class)) {
				return new BoolRemotePullSenderImpl();
			}
			throw new ConnectionCreationException(
					"No implemented Pull connector for primitive type: "
							+ _dataClass);

		}
		else {
			// first create a new generic connector
			GenericRemotePullSender<D> grps = new GenericRemotePullSender<D>(
					_dataClass);

			Method[] listenerMethods = new Method[2];
			Method[] targetMethods = new Method[2];

			listenerMethods[0] = ConnectorGenerator
					.getListenerMethod(_connectorClass,
							PullConnectorOut.PULL_CONNECTOR_OUT_METHOD);
			listenerMethods[1] = ConnectorGenerator.getListenerMethod(
					RemotePullSender.class, PullSender.PULL_SENDER_METHOD);

			targetMethods[0] = ConnectorGenerator.getTargetMethod(grps,
					PullConnectorOut.PULL_CONNECTOR_OUT_METHOD,
					listenerMethods[0].getParameterTypes());
			targetMethods[1] = ConnectorGenerator.getTargetMethod(grps,
					PullSender.PULL_SENDER_METHOD, listenerMethods[1]
							.getParameterTypes());

			// now generate an object to wrap it in
			Object pcls = ConnectorGenerator.create(RemotePullSender.class,
					listenerMethods, grps, targetMethods);

			return pcls;
		}

	}

	/**
	 * @param _dataType
	 * @param _recvClass
	 * @param _connectorClass
	 * @return
	 * @throws ConnectionCreationException
	 */
	private static <D, C> Object generateRemotePushSender(Class<D> _data,
			Class<C> _connectorClass) throws ConnectionCreationException {

		if (_data.isPrimitive()) {

			if (_data.equals(int.class)) {
				return new IntRemotePushSenderImpl();
			}
			else if (_data.equals(long.class)) {
				return new LongRemotePushSenderImpl();
			}
			else if (_data.equals(short.class)) {
				return new ShortRemotePushSenderImpl();
			}
			else if (_data.equals(double.class)) {
				return new DoubleRemotePushSenderImpl();
			}
			else if (_data.equals(float.class)) {
				return new FloatRemotePushSenderImpl();
			}
			else if (_data.equals(char.class)) {
				return new CharRemotePushSenderImpl();
			}
			else if (_data.equals(byte.class)) {
				return new ByteRemotePushSenderImpl();
			}
			else if (_data.equals(boolean.class)) {
				return new BoolRemotePushSenderImpl();
			}
			throw new ConnectionCreationException(
					"No implemented push connector for primitive type: "
							+ _data);

		}
		else {
			// first create a new generic connector
			GenericRemotePushSender<D> grpc = new GenericRemotePushSender<D>(
					_data);

			Method[] listenerMethods = new Method[3];
			Method[] targetMethods = new Method[3];

			listenerMethods[0] = ConnectorGenerator
					.getListenerMethod(_connectorClass,
							PushConnectorOut.PUSH_CONNECTOR_OUT_METHOD);

			listenerMethods[1] = ConnectorGenerator.getListenerMethod(
					RemotePushSender.class, PushSender.PUSH_SENDER_METHOD);

			// flush addition
			listenerMethods[2] = ConnectorGenerator.getListenerMethod(
					_connectorClass, "flush");

			targetMethods[0] = ConnectorGenerator.getTargetMethod(grpc,
					PushConnectorOut.PUSH_CONNECTOR_OUT_METHOD,
					listenerMethods[0].getParameterTypes());
			targetMethods[1] = ConnectorGenerator.getTargetMethod(grpc,
					PushSender.PUSH_SENDER_METHOD, listenerMethods[1]
							.getParameterTypes());

			// flush addition
			targetMethods[2] = ConnectorGenerator.getTargetMethod(grpc,
					"flush", listenerMethods[2].getParameterTypes());

			// now generate an object to wrap it in
			Object pcls = ConnectorGenerator.create(RemotePushSender.class,
					listenerMethods, grpc,
					// _connectorClass, listenerMethods, grpc,
					targetMethods);

			return pcls;
		}

	}

	public static void cleanup() {
		for (RemotePushConnectorImpl rpci : m_pushConnectors) {
			rpci.stopConnector();
		}
	}

	/**
	 * @param _desc
	 * @param _processB
	 */
	@SuppressWarnings("unchecked")
	public static void connectToRemotePullConnector(
			ConnectionDescription _desc, PullReceiver _processB) {
		try {
			// first task is to find the object to connect to
			// get the root naming context
			org.omg.CORBA.Object objRef = m_orb
					.resolve_initial_references("NameService");

			// Use NamingContextExt instead of NamingContext. This is
			// part of the Interoperable naming Service.
			NamingContextExt ncRef = NamingContextExtHelper.narrow(objRef);

			// resolve the Object Reference in Naming
			String name = generateServerName(_desc);
			// System.out.println("server name: " + name);

			// ClientUtils.browseNaming(getORB(),ncRef);

			RemotePullConnector remotePullConnector = RemotePullConnectorHelper
					.narrow(ncRef.resolve_str(name));

			// System.out.println("Obtained a handle on server object: "
			// + remotePushConnector);

			POA rootPOA = POAHelper.narrow(m_orb
					.resolve_initial_references("RootPOA"));
			// System.out.println("RootPOA narrowed");

			Class<?> dataClass = LocalConnectionFactory.lookupClass(_desc
					.getDataType());

			if (dataClass == null) {
				throw new FrameworkConnectionException(
						"Unable to find data type " + _desc.getDataType()
								+ " in LocalConnectionFactory");
			}
			// Class<?> connectorClass = LocalConnectionFactory
			// .lookupPullConnector(dataClass);

			// Now find the correct receiver interface
			Class<?> recvClass = LocalConnectionManager
					.getReceiverClassForPull(_processB.getClass(), dataClass);
			// set up a local adapter for the process

			RemotePullReceiverImpl<?, ?> receiverAdaptor = new RemotePullReceiverImpl(
					recvClass, dataClass);

			rootPOA.activate_object(receiverAdaptor);

			// narrow to a useful reference
			RemotePullReceiver ref = RemotePullReceiverHelper.narrow(rootPOA
					.servant_to_reference(receiverAdaptor));

			// remote pushes to local adaptor
			remotePullConnector.registerPullReceiver(ref);

			LocalConnectionManager.connect(
					(PullConnectorRegister) receiverAdaptor, receiverAdaptor
							.getClass(), _processB, recvClass);

		}
		catch (NotFound e) {
			// e.printStackTrace();
			System.err.println(e.getLocalizedMessage());
			System.err.println(e.why.value());
			System.err.println(e.rest_of_name[0].id);
			System.exit(1);
		}
		catch (CannotProceed e) {
			e.printStackTrace();
			System.exit(1);
		}
		catch (InvalidName e) {
			e.printStackTrace();
			System.exit(1);
		}
		catch (Throwable t) {
			System.err.println("Error here...");
			t.printStackTrace();
			System.exit(1);
		}

		// System.out.println("Connection appears to be made OK!");

	}

	/**
	 * Connect the input process to a connection objects constructed remotely.
	 * 
	 * @param _desc
	 *            The description of the connection. Used to find the server.
	 * @param _recv
	 *            The process to connect to the server.
	 * @throws ServantNotActive
	 */
	@SuppressWarnings("unchecked")
	public static void connectToRemotePushConnector(
			ConnectionDescription _desc, PushReceiver _recv) {
		try {
			// first task is to find the object to connect to
			// get the root naming context
			org.omg.CORBA.Object objRef = m_orb
					.resolve_initial_references("NameService");

			// Use NamingContextExt instead of NamingContext. This is
			// part of the Interoperable naming Service.
			NamingContextExt ncRef = NamingContextExtHelper.narrow(objRef);

			// resolve the Object Reference in Naming
			String name = generateServerName(_desc);
			// System.out.println("server name: " + name);

			// ClientUtils.browseNaming(getORB(),ncRef);

			RemotePushConnector remotePushConnector = RemotePushConnectorHelper
					.narrow(ncRef.resolve_str(name));

			// System.out.println("Obtained a handle on server object: "
			// + remotePushConnector);

			POA rootPOA = POAHelper.narrow(m_orb
					.resolve_initial_references("RootPOA"));
			// System.out.println("RootPOA narrowed");

			Class<?> dataClass = LocalConnectionFactory.lookupClass(_desc
					.getDataType());

			if (dataClass == null) {
				throw new FrameworkConnectionException(
						"Unable to find data type " + _desc.getDataType()
								+ " in LocalConnectionFactory");
			}

			Class<?> connectorClass = LocalConnectionFactory
					.lookupPushConnector(dataClass);

			if (connectorClass == null) {
				throw new FrameworkConnectionException(
						"Unable to find push connector type for " + dataClass
								+ " in LocalConnectionFactory");
			}

			// Now find the correct receiver interface
			Class<?> recvClass = LocalConnectionManager
					.getReceiverClassForPush(_recv.getClass(), dataClass);

			// set up a local adapter for the process
			RemotePushReceiverImpl<?, ?> receiverAdaptor = new RemotePushReceiverImpl(
					recvClass, dataClass);
			rootPOA.activate_object(receiverAdaptor);

			// narrow to a useful reference
			RemotePushReceiver ref = RemotePushReceiverHelper.narrow(rootPOA
					.servant_to_reference(receiverAdaptor));

			// remote pushes to local adaptor
			remotePushConnector.registerPushReceiver(ref);

			// local adaptor pushes to process
			// receiverAdaptor.registerPushReceiver(_recv);

			LocalConnectionManager.connect(
					(PushConnectorRegister) receiverAdaptor, receiverAdaptor
							.getClass(), _recv, recvClass);

		}
		catch (NotFound e) {
			// e.printStackTrace();
			System.err.println(e.getLocalizedMessage());
			System.err.println(e.why.value());
			System.err.println(e.rest_of_name[0].id);
			System.exit(1);
		}
		catch (CannotProceed e) {
			e.printStackTrace();
			System.exit(1);
		}
		catch (InvalidName e) {
			e.printStackTrace();
			System.exit(1);
		}
		catch (Throwable t) {
			System.err.println("Error here...");
			t.printStackTrace();
			System.exit(1);
		}

		// System.out.println("Connection appears to be made OK!");

	}

	/**
	 * Creates a remote connection objects and connects the given process to it.
	 * 
	 * @param _desc
	 *            The description of the connection. Used to name the connection
	 *            object.
	 * @param _sender
	 *            The sender object to connect to the server/
	 * @throws FrameworkConnectionException
	 */
	public static String createRemotePullConnectionServer(
			ConnectionDescription _desc) throws FrameworkConnectionException {
		try {

			// System.out.println("RemoteConnectionManager.createRemotePushConnectionServer()");

			// Get reference to rootpoa & activate the POAManager
			POA rootPOA = POAHelper.narrow(m_orb
					.resolve_initial_references("RootPOA"));

			String name = generateServerName(_desc);

			POA childPOA = rootPOA.find_POA(name, true);
			// System.out.println("found new POA: " + childPOA);

			// Create the object reference for servant
			// and register with naming service
			org.omg.CORBA.Object obj = childPOA
					.id_to_reference(name.getBytes());

			RemotePullConnector rpc = RemotePullConnectorHelper.narrow(obj);

			NamingContext namingContext = NamingContextHelper.narrow(getORB()
					.resolve_initial_references("NameService"));
			NameComponent[] nc = {new NameComponent(name, "")};
			namingContext.rebind(nc, rpc);

			return name;

		}
		catch (Exception e) {
			throw new FrameworkConnectionException(
					"Unable to create remote server for connection: " + _desc,
					e);
		}

	}

	/**
	 * Creates a remote connection objects and connects the given process to it.
	 * 
	 * @param _desc
	 *            The description of the connection. Used to name the connection
	 *            object.
	 * @param _sender
	 *            The sender object to connect to the server/
	 * @throws FrameworkConnectionException
	 */
	public static void createRemotePullConnectionServer(
			ConnectionDescription _desc, PullSender _sender)
			throws FrameworkConnectionException {
		try {

			// System.out
			// .println("RemoteConnectionManager.createRemotePullConnectionServer()");

			// Get reference to rootpoa & activate the POAManager
			POA rootPOA = POAHelper.narrow(m_orb
					.resolve_initial_references("RootPOA"));

			String name = generateServerName(_desc);

			POA childPOA = rootPOA.find_POA(name, true);
			// System.out.println("found new POA: " + childPOA);

			// Create the object reference for servant
			// and register with naming service
			org.omg.CORBA.Object obj = childPOA
					.id_to_reference(name.getBytes());

			RemotePullConnector rpc = RemotePullConnectorHelper.narrow(obj);

			NamingContext namingContext = NamingContextHelper.narrow(getORB()
					.resolve_initial_references("NameService"));

			NameComponent[] nc = {new NameComponent(name, "")};
			namingContext.rebind(nc, rpc);

			connectToRemotePullConnector(_desc, _sender);

			// System.out.println(name + " ready and waiting ...");

		}
		catch (Exception e) {
			throw new FrameworkConnectionException(
					"Unable to create remote server for connection: " + _desc,
					e);
		}

	}

	/**
	 * @param _desc
	 * @param _sender
	 * @param rootPOA
	 * @param rpc
	 * @throws FrameworkConnectionException
	 * @throws ConnectionCreationException
	 */
	public static void connectToRemotePullConnector(
			ConnectionDescription _desc, PullSender _sender)
			throws FrameworkConnectionException, ConnectionCreationException {
		try {
			// first task is to find the object to connect to
			// get the root naming context
			org.omg.CORBA.Object objRef = m_orb
					.resolve_initial_references("NameService");

			// Use NamingContextExt instead of NamingContext. This is
			// part of the Interoperable naming Service.
			NamingContextExt ncRef = NamingContextExtHelper.narrow(objRef);

			// resolve the Object Reference in Naming
			String name = generateServerName(_desc);
			// System.out.println("server name: " + name);

			// ClientUtils.browseNaming(getORB(),ncRef);

			RemotePullConnector remotePullConnector = RemotePullConnectorHelper
					.narrow(ncRef.resolve_str(name));

			// System.out.println("Obtained a handle on server object: "
			// + remotePushConnector);

			POA rootPOA = POAHelper.narrow(m_orb
					.resolve_initial_references("RootPOA"));
			// System.out.println("RootPOA narrowed");

			Class<?> dataClass = LocalConnectionFactory.lookupClass(_desc
					.getDataType());

			if (dataClass == null) {
				throw new FrameworkConnectionException(
						"Unable to find data type " + _desc.getDataType()
								+ " in LocalConnectionFactory");
			}

			Class<?> connectorClass = LocalConnectionFactory
					.lookupPullConnector(dataClass);

			if (connectorClass == null) {
				throw new FrameworkConnectionException(
						"Unable to find pull connector type for " + dataClass
								+ " in LocalConnectionFactory");
			}

			// set up a local adapter for the process
			Object senderAdaptor = generateRemotePullSender(dataClass,
					connectorClass);

			if (senderAdaptor instanceof Servant) {
				try {
					RemotePullSender ref = RemotePullSenderHelper
							.narrow(rootPOA
									.servant_to_reference((Servant) senderAdaptor));
					// the senderAdaptor Pulles into the server
					ref.setPullConnector(remotePullConnector);
				}
				catch (ServantNotActive e) {
					e.printStackTrace();
					System.exit(1);
				}
				catch (WrongPolicy e) {
					e.printStackTrace();
					System.exit(1);
				}
			}
			else {
				((RemotePullSender) senderAdaptor)
						.setPullConnector(remotePullConnector);
			}

			Class<?> senderClass = LocalConnectionManager
					.getSenderClassForPull(_sender.getClass(), connectorClass);

			// // local process pushes into the senderAdaptor
			LocalConnectionManager.connect(_sender, senderClass,
					(PullConnectorOut) senderAdaptor, connectorClass, _desc
							.getId());

		}
		catch (NotFound e) {
			// e.printStackTrace();
			System.err.println(e.getLocalizedMessage());
			System.err.println(e.why.value());
			System.err.println(e.rest_of_name[0].id);
			System.exit(1);
		}
		catch (CannotProceed e) {
			e.printStackTrace();
			System.exit(1);
		}
		catch (InvalidName e) {
			e.printStackTrace();
			System.exit(1);
		}
		catch (Throwable t) {
			System.err.println("Error here...");
			t.printStackTrace();
			System.exit(1);
		}

	}

	/**
	 * Creates the server and returns the name of the object from the naming
	 * service. Doesn't do any connection. Used when creating connection objects
	 * for non-Java sender processes.
	 * 
	 * @param _desc
	 *            The description of the connection. Used to name the connection
	 *            object.
	 * @return The name under which the connection object is bound in the naming
	 *         service.
	 * @throws FrameworkConnectionException
	 */
	public static String createRemotePushConnectionServer(
			ConnectionDescription _desc) throws FrameworkConnectionException {
		try {

			// System.out.println("RemoteConnectionManager.createRemotePushConnectionServer()");

			// Get reference to rootpoa & activate the POAManager
			POA rootPOA = POAHelper.narrow(m_orb
					.resolve_initial_references("RootPOA"));

			String name = generateServerName(_desc);

			POA childPOA = rootPOA.find_POA(name, true);

			// Create the object reference for servant
			// and register with naming service
			org.omg.CORBA.Object obj = childPOA
					.id_to_reference(name.getBytes());

			RemotePushConnector rpc = RemotePushConnectorHelper.narrow(obj);

			NamingContext namingContext = NamingContextHelper.narrow(getORB()
					.resolve_initial_references("NameService"));

			NameComponent[] nc = {new NameComponent(name, "")};
			namingContext.rebind(nc, rpc);

			// System.out.println(name + " ready and waiting ...");
			return name;

		}
		catch (Exception e) {
			throw new FrameworkConnectionException(
					"Unable to create remote server for connection: " + _desc,
					e);
		}
	}

	// /**
	// * Creates a remote connection objects and connects the given
	// * process to it.
	// *
	// * @param _desc
	// * The description of the connection. Used to name the
	// * connection object.
	// * @param _sender
	// * The sender object to connect to the server/
	// * @throws FrameworkConnectionException
	// */
	// public static void createRemotePushConnectionServer(ConnectionDescription
	// _desc,
	// PushSender _sender)
	// throws FrameworkConnectionException {
	// try {
	//
	// //
	// System.out.println("RemoteConnectionManager.createRemotePushConnectionServer()");
	//
	// // Get reference to rootpoa & activate the POAManager
	// POA rootPOA =
	// POAHelper.narrow(m_orb
	// .resolve_initial_references("RootPOA"));
	//
	// String name = generateServerName(_desc);
	//
	// POA childPOA = rootPOA.find_POA(name, true);
	// // System.out.println("found new POA: " + childPOA);
	//
	// // Create the object reference for servant
	// // and register with naming service
	// org.omg.CORBA.Object obj =
	// childPOA.id_to_reference(name.getBytes());
	//
	// RemotePushConnector rpc =
	// RemotePushConnectorHelper.narrow(obj);
	//
	// NamingContext namingContext =
	// NamingContextHelper.narrow(getORB()
	// .resolve_initial_references("NameService"));
	//
	// NameComponent[] nc = {
	// new NameComponent(name, "")
	// };
	// namingContext.rebind(nc, rpc);
	//
	// connectToRemotePushConnector(_desc, _sender);
	//
	// // System.out.println(name + " ready and waiting ...");
	// }
	// catch (Exception e) {
	// e.printStackTrace();
	// System.exit(1);
	// throw new FrameworkConnectionException(
	// "Unable to create remote server for connection: "
	// + _desc, e);
	// }
	// }

	/**
	 * @param _desc
	 * @param _sender
	 * @param rootPOA
	 * @param rpc
	 * @throws FrameworkConnectionException
	 * @throws ConnectionCreationException
	 */
	public static void connectToRemotePushConnector(
			ConnectionDescription _desc, PushSender _sender)
			throws FrameworkConnectionException, ConnectionCreationException {

		try {

			// System.out
			// .println("RemoteConnectionManager.connectToRemotePushConnector():
			// " + _desc);

			// first task is to find the object to connect to
			// get the root naming context
			org.omg.CORBA.Object objRef = m_orb
					.resolve_initial_references("NameService");

			// Use NamingContextExt instead of NamingContext. This is
			// part of the Interoperable naming Service.
			NamingContextExt ncRef = NamingContextExtHelper.narrow(objRef);

			// resolve the Object Reference in Naming
			String name = generateServerName(_desc);
			// System.out.println("server name: " + name);

			// ClientUtils.browseNaming(getORB(),ncRef);

			RemotePushConnector remotePushConnector = RemotePushConnectorHelper
					.narrow(ncRef.resolve_str(name));

			// System.out.println("Obtained a handle on server object: "
			// + remotePushConnector);

			POA rootPOA = POAHelper.narrow(m_orb
					.resolve_initial_references("RootPOA"));
			// System.out.println("RootPOA narrowed");

			Class<?> dataClass = LocalConnectionFactory.lookupClass(_desc
					.getDataType());

			if (dataClass == null) {
				throw new FrameworkConnectionException(
						"Unable to find data type " + _desc.getDataType()
								+ " in LocalConnectionFactory");
			}
			Class<?> connectorClass = LocalConnectionFactory
					.lookupPushConnector(dataClass);

			if (connectorClass == null) {
				throw new FrameworkConnectionException(
						"Unable to find push connector type for " + dataClass
								+ " in LocalConnectionFactory");
			}

			// set up a local adapter for the process
			Object senderAdaptor = generateRemotePushSender(dataClass,
					connectorClass);

			if (senderAdaptor instanceof Servant) {
				try {

					RemotePushSender ref = RemotePushSenderHelper
							.narrow(rootPOA
									.servant_to_reference((Servant) senderAdaptor));
					// the senderAdaptor pushes into the server
					ref.setPushConnector(remotePushConnector);
				}
				catch (ServantNotActive e) {
					e.printStackTrace();
					System.exit(1);
				}
				catch (WrongPolicy e) {
					e.printStackTrace();
					System.exit(1);
				}

			}
			else {
				((RemotePushSender) senderAdaptor)
						.setPushConnector(remotePushConnector);
			}

			Class<?> senderClass = LocalConnectionManager
					.getSenderClassForPush(_sender.getClass(), connectorClass);

			// local process pushes into the senderAdaptor
			// _sender.setPushConnector(_desc.getId(), senderAdaptor);
			LocalConnectionManager.connect(_sender, senderClass,
					(PushConnectorOut) senderAdaptor, connectorClass, _desc
							.getId());
		}
		catch (NotFound e) {
			// e.printStackTrace();
			System.err.println(e.getLocalizedMessage());
			System.err.println(e.why.value());
			System.err.println(e.rest_of_name[0].id);
			System.exit(1);
		}
		catch (CannotProceed e) {
			e.printStackTrace();
			System.exit(1);
		}
		catch (InvalidName e) {
			e.printStackTrace();
			System.exit(1);
		}
		catch (Throwable t) {
			System.err.println("Error here...");
			t.printStackTrace();
			System.exit(1);
		}
	}

	private static String processNames(ManagedProcessDescription[] _descs) {
		String namesString = "__";
		for (int i = 0; i < _descs.length; i++) {
			namesString += _descs[i].getProcessName() + "__";
		}
		return namesString;
	}

	/**
	 * Generate a server name from a connection description. Also replaces all
	 * dots with underscores as they mess with the naming service.
	 * 
	 * @param _desc
	 *            The connection description.
	 * @return The server name.
	 */
	public static String generateServerName(ConnectionDescription _desc) {

		if (_desc.isPushConnection()) {
			String sName = PUSH_CONNECTOR_PREFIX + ":" + _desc.getDataType()
					+ ":" + processNames(_desc.getProcessDescriptionA()) + ":"
					+ processNames(_desc.getProcessDescriptionB());
			// sName.re
			return sName.replaceAll("\\.", "_");
		}
		if (_desc.isPullConnection()) {
			String sName = PULL_CONNECTOR_PREFIX + ":" + _desc.getDataType()
					+ ":" + processNames(_desc.getProcessDescriptionA()) + ":"
					+ processNames(_desc.getProcessDescriptionB());
			return sName.replaceAll("\\.", "_");
		}

		return "RemoteServer";
	}

	/**
	 * Get the ORB used by the connection manager. This is the only ORB that
	 * will be used by the process server.
	 * 
	 * @return The remote connection manager's ORB.
	 */
	public static ORB getORB() {
		return m_orb;
	}

	/**
	 * Initialise the remote connection manager's ORB.
	 * 
	 * @param args
	 *            Init arguments (from command line).
	 * @param props
	 *            Init properties.
	 */
	public static void init(String[] _args, Properties _props) {
		m_orb = ORB.init(_args, _props);

		m_namingHost = (String) _props.get("org.omg.CORBA.ORBInitialHost");
		m_namingPort = (String) _props.get("org.omg.CORBA.ORBInitialPort");
	}

	public static void main(String[] args) {
		System.out
				.println(new String("124.3.ee.asrdsf").replaceAll("\\.", "_"));
	}

	public static Any nullAny() {
		if (m_nullAny == null) {
			m_nullAny = m_orb.create_any();
		}

		return m_nullAny;
	}

	/**
	 * Run the ORB to wait for connections.
	 */
	public static void run() {
		m_orb.run();
	}

	/**
	 * @return Returns the m_namingHost.
	 */
	public static String getNamingHost() {
		return m_namingHost;
	}

	/**
	 * @return Returns the m_namingPort.
	 */
	public static String getNamingPort() {
		return m_namingPort;
	}

}
