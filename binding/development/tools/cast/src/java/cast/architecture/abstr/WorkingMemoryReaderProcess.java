/*
 * cast - the CoSy Architecture Schema Toolkit Copyright (C) 2006-2007
 * Nick Hawes This library is free software; you can redistribute it
 * and/or modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either version
 * 2.1 of the License, or (at your option) any later version. This
 * library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. You should have
 * received a copy of the GNU Lesser General Public License along with
 * this library; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * 
 */
package cast.architecture.abstr;

import java.util.LinkedList;
import java.util.Queue;
import java.util.Vector;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Semaphore;

import balt.core.connectors.ControlledRunnable;
import balt.core.connectors.pull.primitive.interfaces.IntPullInterface.IntPullSender;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.SubarchitectureWorkingMemoryProtocol;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.ui.ComponentEventType;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import cast.core.interfaces.WMCFPushInterface.WMCFPushConnectorOut;
import cast.core.interfaces.WMCFPushInterface.WMCFPushSender;
import cast.core.interfaces.WMCPushInterface.WMCPushReceiver;

/**
 * Defines a class of process that can read from any working memory. This
 * process is based largely on receiving change events that inform the process
 * about what can be read. These change events can be filtered. When no filters
 * are added all events are allowed through. When filters are added, only the
 * events matched by the filters are allowed through.
 * 
 * @author nah
 */
public abstract class WorkingMemoryReaderProcess
		extends
			WorkingMemoryAttachedComponent
		implements
			WMCPushReceiver,
			WMCFPushSender,
			IntPullSender {

	/**
	 * Inner class used to propagate change information to a derived class in a
	 * separate thread.
	 * 
	 * @author nah
	 */
	private class WorkingMemoryChangeThread extends ControlledRunnable {

		/**
		 * The stored change events.
		 */
		private Queue<WorkingMemoryChange> m_changes;

		/**
		 * The process that the events will be forwarded on to.
		 */
		private WorkingMemoryReaderProcess m_gdp;

		private Semaphore m_changeSemaphore;

		/**
		 * Helper to hold receivers before use
		 */
		private final Queue<WorkingMemoryChangeReceiver> m_receivers;

		/**
		 * Create a new thread to forward events to the input object.
		 * 
		 * @param _gdp
		 *            The process that the events will be forwarded on to.
		 */
		public WorkingMemoryChangeThread(WorkingMemoryReaderProcess _gdp) {
			// m_changes = new
			// ConcurrentLinkedQueue<WorkingMemoryChange>();
			m_gdp = _gdp;

			if (m_gdp.m_queueBehaviour == WorkingMemoryChangeQueueBehaviour.DISCARD) {
				m_changes = new SinglePlaceQueue<WorkingMemoryChange>();
			}
			else if (m_gdp.m_queueBehaviour == WorkingMemoryChangeQueueBehaviour.QUEUE) {
				m_changes = new ConcurrentLinkedQueue<WorkingMemoryChange>();
			}

			m_changeSemaphore = new Semaphore(0, true);
			m_receivers = new LinkedList<WorkingMemoryChangeReceiver>();
		}

		/**
		 * @param changeList
		 * @throws SubarchitectureProcessException
		 */
		private boolean forwardToSubclass(WorkingMemoryChange[] changeList)
				throws SubarchitectureProcessException {

			boolean used = false;
			try {
				// if non null then we should generate event objects
				if (m_gdp.m_changeObjects != null) {

					// remove any previously flagged filters
					removeChangeFilters();

					for (WorkingMemoryChange wmc : changeList) {
						// // get the object for the change
						// WorkingMemoryChangeReceiver receiver =
						// m_gdp.m_changeObjects
						// .get(new OLDWorkingMemoryChangeFilter(
						// wmc.m_type, wmc.m_operation));
						// if (receiver != null) {
						// receiver.workingMemoryChanged(wmc);
						// }
						// else {
						// throw new SubarchitectureProcessException(
						// "Missing change object for filtered change: "
						// + wmc.m_type + " " + wmc.m_operation);
						// }
						//                    
						// get the object for the change

						m_gdp.debug("CHANGE: " + CASTUtils.toString(wmc));

						m_gdp.m_changeObjects.get(wmc, m_receivers);

						if (!m_receivers.isEmpty()) {
							for (WorkingMemoryChangeReceiver receiver : m_receivers) {
								try {
									receiver.workingMemoryChanged(wmc);
									used = true;
								}
								catch (RuntimeException e) {
									e.printStackTrace();
									System.exit(1);
								}
							}
							m_receivers.clear();
						}
						else {
							// no receiver registered for change
							m_gdp.debug("no receiver");
						}

						// remove filters flagged by that change
						removeChangeFilters();

					}

				}
			}
			catch (java.util.ConcurrentModificationException e) {
				log(e.getStackTrace());
				e.printStackTrace();
				System.exit(0);
				throw e;
			}

			// this looks like a good place to give up control
			Thread.yield();

			return used;
		}

		private final void removeChangeFilters()
				throws SubarchitectureProcessException {
			if (!m_gdp.m_receiversToRemove.isEmpty()) {
				for (WorkingMemoryChangeReceiver receiver : m_gdp.m_receiversToRemove) {
					debug("removing receiver: " + receiver);
					m_gdp.removeChangeFilterHelper(receiver);
				}
				m_gdp.m_receiversToRemove.clear();
			}
		}

		/**
		 * Run the thread, discarding any events that are received whilst the
		 * derived class's thread is locked.
		 * 
		 * @throws SubarchitectureProcessException
		 */
		private void runDiscard() throws SubarchitectureProcessException {
			WorkingMemoryChange[] changeList = new WorkingMemoryChange[1];
			boolean used = false;
			while (isRunning()) {

				while (isRunning() && !m_changes.isEmpty()) {

					// try to gain access
					// if (m_gdp.m_semaphore.tryAcquire()) {
					m_gdp.lockProcess();

					// get changes from queue

					synchronized (m_changes) {
						if (m_changes.size() > 1) {
							m_gdp.debug("discarding " + (m_changes.size() - 1)
									+ " change events ");
						}
						else {
							// m_gdp.log("forwarding 1 event");
						}

						changeList[0] = m_changes.poll();
						m_changes.clear();

					}

					try {
						// write to derived class
						used = forwardToSubclass(changeList);
					}
					catch (SubarchitectureProcessException e) {
						println(e.getLocalizedMessage());
						e.printStackTrace();
					}

					synchronized (m_gdp.m_componentStatus) {
						m_gdp.m_componentStatus.m_changeQueue = 0;
					}

					// unlock derived class now something has
					// been
					// done
					m_gdp.unlockProcess();

					// cout<<m_pWMRP->getProcessIdentifier()<<":
					// "<<" RUN DONE "<<endl;

					// }
					// else {
					// m_gdp.log("unable to access thread");
					// synchronized (m_gdp.m_unlockNotification) {
					// while (isRunning()
					// && m_gdp.m_semaphore.availablePermits() == 0)
					// {
					// try {
					// m_gdp.m_unlockNotification.wait();
					// }
					// catch (InterruptedException e) {
					// e.printStackTrace();
					// System.exit(1);
					// }
					// }
					// }
					// }

				}

				if (used) {
					synchronized (m_gdp.m_wmChangeLock) {
						// now wake up any waiting threads
						m_gdp.m_wmChangeLock.notifyAll();
					}
				}
				try {
					m_changeSemaphore.acquire();
				}
				catch (InterruptedException e) {
					e.printStackTrace();
					System.exit(1);
				}

			}
		}

		/**
		 * Run the thread, queuing any events that are received whilst the
		 * derived class's thread is locked.
		 * 
		 * @throws SubarchitectureProcessException
		 */
		private void runQueue() throws SubarchitectureProcessException {

			WorkingMemoryChange[] changeList;
			boolean used = false;

			while (isRunning()) {

				while (!m_changes.isEmpty()) {

					// try to gain access
					// if (m_gdp.m_semaphore.tryAcquire()) {
					m_gdp.lockProcess();
					synchronized (m_changes) {
						// log(m_changes.size());
						changeList = new WorkingMemoryChange[m_changes.size()];
						// log(m_changes.size());
						for (int i = 0; i < changeList.length; i++) {
							changeList[i] = m_changes.poll();
						}
						// log(m_changes.size());
						assert m_changes.size() == 0 : "emptied change list is not empty!";

						// if (changeList.length > 1) {
						// System.out.print("list longer than 1...
						// ");
						// }

					}
					try {
						// write to derived class
						used = forwardToSubclass(changeList);
					}
					catch (SubarchitectureProcessException e) {
						println(e.getLocalizedMessage());
						e.printStackTrace();
					}
					// if (changeList.length > 1) {
					// System.out.println("done");
					// }

					synchronized (m_gdp.m_componentStatus) {
						m_gdp.m_componentStatus.m_changeQueue = 0;
					}

					// unlock derived class now something has
					// been
					// done
					m_gdp.unlockProcess();

					if (used) {
						synchronized (m_gdp.m_wmChangeLock) {
							// now wake up any waiting threads
							m_gdp.m_wmChangeLock.notifyAll();
						}
					}
					// cout<<m_pWMRP->getProcessIdentifier()<<":
					// "<<" RUN DONE "<<endl;

					// }
					// else {
					// m_gdp.println("unable to access thread");
					// synchronized (m_gdp.m_unlockNotification) {
					// while (isRunning()
					// && m_gdp.m_semaphore.availablePermits() == 0)
					// {
					// try {
					// m_gdp.m_unlockNotification.wait();
					// }
					// catch (InterruptedException e) {
					// e.printStackTrace();
					// System.exit(1);
					// }
					// }
					// }
					//
					// }
				}
				// synchronized (this) {
				// // check we should be running before sleep
				// while (isRunning() && m_changes.isEmpty()) {
				//
				// try {
				// wait();
				// }
				// catch (InterruptedException e) {
				// e.printStackTrace();
				// System.exit(1);
				// }
				// }
				// }

				try {
					m_changeSemaphore.acquire();
				}
				catch (InterruptedException e) {
					e.printStackTrace();
					System.exit(1);
				}

			}
		}

		/**
		 * Add change structs to the queue for forwarding.
		 * 
		 * @param _changes
		 *            A list of change structs.
		 */
		public void queueChange(WorkingMemoryChange _change) {

			// synchronized (this) {

			synchronized (m_changes) {
				m_changes.add(_change);
			}

			synchronized (m_gdp.m_componentStatus) {
				m_gdp.m_componentStatus.m_changeQueue++;
			}

			m_changeSemaphore.release();

			// notify();
			// }

		}

		/**
		 * Run thread and send changes to object. This method calls either
		 * runDiscard or runQueue based on the value of m_queueBehaviour in the
		 * target class.
		 */
		public void run() {
			try {
				if (m_gdp.m_queueBehaviour == WorkingMemoryChangeQueueBehaviour.DISCARD) {
					m_changes = new SinglePlaceQueue<WorkingMemoryChange>();
					runDiscard();
				}
				else if (m_gdp.m_queueBehaviour == WorkingMemoryChangeQueueBehaviour.QUEUE) {
					m_changes = new ConcurrentLinkedQueue<WorkingMemoryChange>();
					runQueue();
				}
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}

		/**
		 * Prevent the thread from processing.
		 */
		public void stop() {

			super.stop();

			synchronized (m_changes) {
				m_changes.clear();
			}

			// synchronized (this) {
			// notify();
			// }

			m_changeSemaphore.release();
		}

	}

	private WMCFPushConnectorOut m_filterOutput;

	private Object m_wmChangeLock;

	/**
	 * The thread that is used to forward change events to the derived class.
	 */
	private WorkingMemoryChangeThread m_wmChangeRunnable;

	/**
	 * The thread that is used to forward change events to the derived class.
	 */
	private Thread m_wmChangeThread;

	private Vector<WorkingMemoryChangeReceiver> m_receiversToRemove;

	protected boolean m_bReceivingChanges;

	private WorkingMemoryChangeFilterMap<WorkingMemoryChangeReceiver> m_changeObjects;

	//
	// /**
	// * Flag to determine whether this component should receive change events
	// * from other subarchitectures.
	// */
	// protected boolean m_receiveXarchChangeNotifications;

	/**
	 * Used to determine whether change events are queued or discarded. Default
	 * is to discard them.
	 */
	protected WorkingMemoryChangeQueueBehaviour m_queueBehaviour;

	/**
	 * Construct a new processing component with the given unique ID. Set queue
	 * behaviour to DISCARD, and xarch change event receiving to false.
	 * 
	 * @param _id
	 *            The id used to identify this component.
	 */
	public WorkingMemoryReaderProcess(String _id) {
		super(_id);

		m_versionNumberConnector = null;

		receiveChanges();

		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		m_wmChangeRunnable = new WorkingMemoryChangeThread(this);
		m_wmChangeLock = new Object();

		m_changeObjects = new WorkingMemoryChangeFilterMap<WorkingMemoryChangeReceiver>();
		m_receiversToRemove = new Vector<WorkingMemoryChangeReceiver>();
	}

	/**
	 * @param _filteredCount
	 * @param _totalCount
	 */
	private void logChangeEvents(int _filteredCount, int _totalCount) {
		synchronized (m_componentStatus) {
			m_componentStatus.m_totalChangeEventsFiltered += _filteredCount;
			m_componentStatus.m_totalChangeEventsReceived += _totalCount;
		}
	}

	private void logReadEvent(String _subarch, String _id) {
		logEvent(ComponentEventType.GET, getProcessIdentifier().toString(),
				_subarch, "", _id);
	}

	private void logReadEvent(String _subarch, String _type, int _count) {
		logEvent(ComponentEventType.GET, getProcessIdentifier().toString(),
				_subarch, _type, "");
	}

	protected int getFilterCount() {
		return m_changeObjects.size();
	}

	/**
	 * Remove a specific change receiver. If the list for this receiver is
	 * subsequently empty, remove filter.
	 * 
	 * @param _workingMemoryChangeReceiver
	 * @throws SubarchitectureProcessException
	 */
	private void removeChangeFilterHelper(
			WorkingMemoryChangeReceiver _workingMemoryChangeReceiver)
			throws SubarchitectureProcessException {

		Vector<WorkingMemoryChangeFilter> removed = new Vector<WorkingMemoryChangeFilter>();
		m_changeObjects.remove(_workingMemoryChangeReceiver, removed);

		for (WorkingMemoryChangeFilter filter : removed) {
			filter.m_filterChange = WorkingMemoryOperation.DELETE;
			m_filterOutput.push(getProcessIdentifier(), filter);
		}

	}

	/**
	 * Add a new filter object to receive the change events. This allows change
	 * events of the given type to be received for the given working memory
	 * operation. This is the most general possible method and is called by all
	 * less detailed methods.
	 * 
	 * For helper functions to construct filters, see
	 * {@link ChangeFilterFactory}.
	 * 
	 * @param _receiver
	 *            The receiver object
	 * @throws SubarchitectureProcessException
	 */
	public void addChangeFilter(WorkingMemoryChangeFilter _filter,
			WorkingMemoryChangeReceiver _receiver)
			throws SubarchitectureProcessException {

		assert (m_filterOutput != null);

		// sneaky operation to keep filter consistent without always
		// needing a component for info
		if (_filter.m_restriction == FilterRestriction.LOCAL_SA
				&& _filter.m_address.m_subarchitecture.length() == 0) {
			_filter.m_address.m_subarchitecture = m_subarchitectureID;
		}
		// also make sure subarch/restrictions are correct
		else if (_filter.m_address.m_subarchitecture
				.equals(m_subarchitectureID)
				&& _filter.m_restriction == FilterRestriction.ALL_SA) {
			_filter.m_restriction = FilterRestriction.LOCAL_SA;
		}

		// and set origin
		_filter.m_origin = getProcessIdentifier();

		// if (existingReceiver == null) {

		m_changeObjects.put(_filter, _receiver);

		// println("new filters length: " + m_changeObjects.size());

		m_filterOutput.push(getProcessIdentifier(), _filter);

		m_filterOutput.flush();
	}

	/**
	 * Retrieve the working memory entries matching the given type from the
	 * local subarchitecture working memory.
	 * 
	 * @param _cls
	 *            The class of the entries to be returned.
	 * @param _count
	 *            The number of entries to return. A value of 0 means all
	 *            matching entries.
	 * @return An array of matching entries starting with the most recent.
	 * @throws SubarchitectureProcessException
	 *             Exception is thrown if communication or translation fails.
	 */
	public <Type> CASTData<Type>[] getWorkingMemoryEntries(Class<Type> _cls,
			int _count) throws SubarchitectureProcessException {
		return getWorkingMemoryEntries(m_subarchitectureID, _cls, _count);
	}

	/**
	 * Retrieve the working memory entries matching the given type from the
	 * local subarchitecture working memory.
	 * 
	 * @param _cls
	 *            The class of the entries to be returned.
	 * @return An array of matching entries starting with the most recent.
	 * @throws SubarchitectureProcessException
	 *             Exception is thrown if communication or translation fails.
	 */
	public <Type> CASTData<Type>[] getWorkingMemoryEntries(Class<Type> _cls)
			throws SubarchitectureProcessException {
		return getWorkingMemoryEntries(_cls, 0);
	}

	/**
	 * Retrieve all the working memory entries matching the given type from the
	 * specified subarchitecture working memory.
	 * 
	 * @param _subarch
	 *            The subarchitecture which should be used to locate the working
	 *            memory containing the entries.
	 * @param _cls
	 *            The class of the entries to be returned.
	 * @return An array of matching entries starting with the most recent.
	 * @throws SubarchitectureProcessException
	 *             Exception is thrown if communication or translation fails.
	 */
	public <Type> CASTData<Type>[] getWorkingMemoryEntries(String _subarch,
			Class<Type> _cls) throws SubarchitectureProcessException {

		return getWorkingMemoryEntries(_subarch, _cls, 0);
	}

	/**
	 * Retrieve the working memory entries matching the given type from the
	 * specified subarchitecture working memory.
	 * 
	 * @param _subarch
	 *            The subarchitecture which should be used to locate the working
	 *            memory containing the entries.
	 * @param _cls
	 *            The class of the entries to be returned.
	 * @param _count
	 *            The number of entries to return. A value of 0 means all
	 *            matching entries.
	 * @return An array of matching entries starting with the most recent.
	 * @throws SubarchitectureProcessException
	 *             Exception is thrown if communication or translation fails.
	 */
	public <Type> CASTData<Type>[] getWorkingMemoryEntries(String _subarch,
			Class<Type> _cls, int _count)
			throws SubarchitectureProcessException {

		assert _subarch.length() > 0 : "subarchitecture id must not be empty";

		String type = CASTUtils.typeName(_cls);

		logReadEvent(_subarch, type, _count);

		CASTData<Type>[] wmel = queryWorkingMemory(SubarchitectureWorkingMemoryProtocol
				.createTypeQuery(getProcessIdentifier().toString(),
						m_subarchitectureID, _subarch, type, _count));

		for (CASTData<?> wme : wmel) {
			// store this version number for later consistency checks
			updateVersion(wme.getID(), wme.getVersion());
		}

		return wmel;
	}

	// /**
	// * Retrieve the working memory entries with the given ids from the local
	// * subarchitecture working memory.
	// *
	// * @param _subarch
	// * The subarchitecture which should be used to locate the working
	// * memory containing the entries.
	// * @param _ids
	// * An array of working memory entry ids.
	// * @return An array of matching entries.
	// * @throws SubarchitectureProcessException
	// * Exception is thrown if communication or translation fails.
	// */
	// public CASTData<?>[] getWorkingMemoryEntries(String[] _ids)
	// throws SubarchitectureProcessException {
	// return getWorkingMemoryEntries(_ids, m_subarchitectureID);
	// }
	//
	// /**
	// * Retrieve the working memory entries with the given ids from the
	// specified
	// * subarchitecture working memory.
	// *
	// * @param _ids
	// * An array of working memory entry ids.
	// * @param _subarch
	// * The subarchitecture which should be used to locate the working
	// * memory containing the entries.
	// * @return An array of matching entries.
	// * @throws SubarchitectureProcessException
	// * Exception is thrown if communication or translation fails.
	// */
	//
	// public CASTData<?>[] getWorkingMemoryEntries(String[] _ids,
	// String _subarch) throws SubarchitectureProcessException {
	//
	// if (_ids.length == 0) {
	// throw new SubarchitectureProcessException("no ids specified");
	// }
	//
	// for (String id : _ids) {
	// assert id.length() > 0 : "id must not be empty";
	//
	// }
	// assert _subarch.length() > 0 : "subarchitecture id must not be empty";
	//
	// logReadEvent(_subarch, _ids.toString());
	//
	// CASTData<?>[] wmel =
	// queryWorkingMemory(SubarchitectureWorkingMemoryProtocol
	// .createIDArrayQuery(getProcessIdentifier().toString(),
	// m_subarchitectureID, _subarch, _ids));
	//
	// // I wonder if this really should be cause for exception
	// if (wmel.length < _ids.length) {
	// throw new DoesNotExistOnWMException(
	// "Not all ids found on working memory: " + _ids + " != "
	// + wmel.length);
	// } else if (wmel.length > _ids.length) {
	// // should never happen
	// throw new SubarchitectureProcessException(
	// "Some odd error fetching ids: " + _ids + " != "
	// + wmel.length);
	// }
	//
	// for (CASTData<?> wme : wmel) {
	// //store this version number for later consistency checks
	// updateVersion(wme.getID(), wme.getVersion());
	// }
	//		
	// return wmel;
	// }

	/**
	 * Get the entry from working memory in a particular subarchitecture with
	 * the given id.
	 * 
	 * @param _id
	 *            The id for the entry in working memory.
	 * @param _subarch
	 *            The subarchitecture in which the id is located.
	 * @return The requested entry.
	 * @throws SubarchitectureProcessException
	 *             Exception is thrown if communication or translation fails.
	 */
	public CASTData<?> getWorkingMemoryEntry(String _id, String _subarch)
			throws SubarchitectureProcessException {

		// println("id:sa: " + _id + ":" + _subarch);

		assert _id.length() > 0 : "id must not be empty";
		assert _subarch.length() > 0 : "subarchitecture id must not be empty";

		logReadEvent(_subarch, _id);

		CASTData<?>[] wmel = queryWorkingMemory(SubarchitectureWorkingMemoryProtocol
				.createIDQuery(getProcessIdentifier().toString(),
						m_subarchitectureID, _subarch, _id));

		if (wmel.length == 0) {
			throw new DoesNotExistOnWMException(new WorkingMemoryAddress(_id,
					_subarch), "Entry does not exist on wm. Was looking for "
					+ _subarch + ":" + _id);
		}
		else if (wmel.length > 1) {
			// should never happen
			throw new SubarchitectureProcessException(
					"requested entry count different from returned count: " + 1
							+ " != " + wmel.length + ". Looking for "
							+ _subarch + ":" + _id);
		}

		// store this version number for later consistency checks
		updateVersion(wmel[0].getID(), wmel[0].getVersion());

		return wmel[0];
	}

	/**
	 * Get the entry from working memory in a particular subarchitecture with
	 * the given id.
	 * 
	 * @param _wma
	 *            The address of the thing to get
	 * @return The requested entry.
	 * @throws SubarchitectureProcessException
	 *             Exception is thrown if communication or translation fails.
	 */
	public CASTData<?> getWorkingMemoryEntry(WorkingMemoryAddress _wma)
			throws SubarchitectureProcessException {
		return getWorkingMemoryEntry(_wma.m_id, _wma.m_subarchitecture);
	}

	/**
	 * Set filters to receive changes using filters.
	 */
	protected void receiveChanges() {
		m_bReceivingChanges = true;
	}

	/**
	 * Set filters to receive no changes at all.
	 */
	protected void receiveNoChanges() {
		m_bReceivingChanges = false;
	}

	protected void removeChangeFilter(
			WorkingMemoryChangeReceiver _workingMemoryChangeReceiver)
			throws SubarchitectureProcessException {
		m_receiversToRemove.add(_workingMemoryChangeReceiver);
	}

	/**
	 * Put the process to sleep until new working memory changes are received.
	 * This method will sleep until the workingMemoryChanged function next
	 * <b>exits</b>.
	 */
	protected void waitForChanges() {
		synchronized (m_wmChangeLock) {
			try {
				m_wmChangeLock.wait();
			}
			catch (InterruptedException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}
	}

	/**
	 * Get the entry from working memory with the given id.
	 * 
	 * @param _id
	 *            The id for the entry in working memory.
	 * @return The requested entry.
	 * @throws SubarchitectureProcessException
	 *             Exception is thrown if communication or translation fails.
	 */
	public CASTData<?> getWorkingMemoryEntry(String _id)
			throws SubarchitectureProcessException {
		return getWorkingMemoryEntry(_id, m_subarchitectureID);
	}

	/**
	 * Receives the WorkingMemoryChange list from a sub-architecture working
	 * memory and forwards this change to the underlying class in a new thread.
	 * Also locks access to the receiving thread whilst doing this.
	 * 
	 * @param _src
	 *            The component that sent the data.
	 * @param _data
	 *            The change data.
	 */
	public void receivePushData(String _src, WorkingMemoryChange _data) {

		// println("change info src: " + _src);
		// for (int i = 0; i < _data.length; i++) {
		// println(_data[i].m_src);
		// println(_data[i].m_type);
		// }

		// WorkingMemoryChangeThread changeUpdater = new
		// WorkingMemoryChangeThread(
		// this, _data);
		// lockProcess();
		// // System.out.println("GoalDrivenProcess.receivePushData()");
		// new Thread(changeUpdater).start();
		// unlockProcess();

		// no thread, and miss updates if blocking
		if (m_status == ProcessStatus.RUN && m_bReceivingChanges) {

			// prefer to use change objects
			if (m_changeObjects != null) {
				m_wmChangeRunnable.queueChange(_data);
			}
		}
	}

	public void setPushConnector(String _connectionID, WMCFPushConnectorOut _out) {
		m_filterOutput = _out;

	}

	/**
	 * Start this component running. This overridden method also starts the
	 * encapsulated thread that forwards change information.
	 * 
	 * @see balt.core.processes.FrameworkProcess#start()
	 */
	@Override
	public void start() {
		super.start();

		m_wmChangeRunnable.start();
		m_wmChangeThread = new Thread(m_wmChangeRunnable);

		m_wmChangeThread.setPriority(Thread.NORM_PRIORITY + 1);
		m_wmChangeThread.start();
	}

	/**
	 * Stop this component. This overridden method also stops the encapsulated
	 * thread that forwards change information.
	 * 
	 * @see balt.core.processes.FrameworkProcess#stop()
	 */
	@Override
	public void stop() {
		super.stop();

		// stop change thread
		m_wmChangeRunnable.stop();
		try {
			m_wmChangeThread.join();
		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}

		// wake threads waiting on changes
		synchronized (m_wmChangeLock) {
			m_wmChangeLock.notifyAll();
		}
	}

}
