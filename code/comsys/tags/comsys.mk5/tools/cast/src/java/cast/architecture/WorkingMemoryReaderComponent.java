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
package cast.architecture;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Vector;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Semaphore;

import Ice.Current;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeFilter;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryEntry;
import cast.cdl.WorkingMemoryEntrySeqHolder;
import cast.core.CASTData;
import cast.core.CASTUtils;
import cast.core.ControlledRunnable;
import cast.core.SinglePlaceQueue;
import cast.interfaces._WorkingMemoryReaderComponentOperations;

/**
 * Defines a class of process that can read from any working memory. This
 * process is based largely on receiving change events that inform the process
 * about what can be read. These change events can be filtered. When no filters
 * are added all events are allowed through. When filters are added, only the
 * events matched by the filters are allowed through.
 * 
 * @author nah
 */
public abstract class WorkingMemoryReaderComponent extends
		WorkingMemoryWriterComponent implements
		_WorkingMemoryReaderComponentOperations {

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
		private WorkingMemoryReaderComponent m_gdp;

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
		public WorkingMemoryChangeThread(WorkingMemoryReaderComponent _gdp) {
			// m_changes = new
			// ConcurrentLinkedQueue<WorkingMemoryChange>();
			m_gdp = _gdp;

			if (m_gdp.m_queueBehaviour == WorkingMemoryChangeQueueBehaviour.DISCARD) {
				m_changes = new SinglePlaceQueue<WorkingMemoryChange>();
			} else if (m_gdp.m_queueBehaviour == WorkingMemoryChangeQueueBehaviour.QUEUE) {
				m_changes = new ConcurrentLinkedQueue<WorkingMemoryChange>();
			}

			m_changeSemaphore = new Semaphore(0, true);
			m_receivers = new LinkedList<WorkingMemoryChangeReceiver>();
		}

		/**
		 * @param changeList
		 * @throws SubarchitectureComponentException
		 */
		private boolean forwardToSubclass(WorkingMemoryChange[] changeList)
				throws SubarchitectureComponentException {

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
						// throw new SubarchitectureComponentException(
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
								} catch (RuntimeException e) {
									e.printStackTrace();
									System.exit(1);
								}
							}
							m_receivers.clear();
						} else {
							// no receiver registered for change
							m_gdp.debug("no receiver");
						}

						// remove filters flagged by that change
						removeChangeFilters();

					}

				}
			} catch (java.util.ConcurrentModificationException e) {
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
				throws SubarchitectureComponentException {
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
		 * @throws SubarchitectureComponentException
		 */
		private void runDiscard() throws SubarchitectureComponentException {
			WorkingMemoryChange[] changeList = new WorkingMemoryChange[1];
			boolean used = false;
			while (isRunning()) {

				while (isRunning() && !m_changes.isEmpty()) {

					// try to gain access
					// if (m_gdp.m_semaphore.tryAcquire()) {
					m_gdp.lockComponent();

					// get changes from queue

					synchronized (m_changes) {
						if (m_changes.size() > 1) {
							m_gdp.debug("discarding " + (m_changes.size() - 1)
									+ " change events ");
						} else {
							// m_gdp.log("forwarding 1 event");
						}

						changeList[0] = m_changes.poll();
						m_changes.clear();

					}

					try {
						// write to derived class
						used = forwardToSubclass(changeList);
					} catch (SubarchitectureComponentException e) {
						println(e.getLocalizedMessage());
						e.printStackTrace();
					} catch (RuntimeException e) {
						println("Caught RuntimeException and rethrowing: "
								+ e.getMessage());
						throw e;
					}

					// UPGRADE now gone
					// synchronized (m_gdp.m_componentStatus) {
					// m_gdp.m_componentStatus.m_changeQueue = 0;
					// }

					// unlock derived class now something has
					// been
					// done
					m_gdp.unlockComponent();

					// cout<<m_pWMRP->getComponentID()<<":
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
				} catch (InterruptedException e) {
					e.printStackTrace();
					System.exit(1);
				}

			}
		}

		/**
		 * Run the thread, queuing any events that are received whilst the
		 * derived class's thread is locked.
		 * 
		 * @throws SubarchitectureComponentException
		 */
		private void runQueue() throws SubarchitectureComponentException {

			WorkingMemoryChange[] changeList;
			boolean used = false;

			while (isRunning()) {

				while (!m_changes.isEmpty()) {

					// try to gain access
					// if (m_gdp.m_semaphore.tryAcquire()) {
					m_gdp.lockComponent();
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
					} catch (SubarchitectureComponentException e) {
						println(e.getLocalizedMessage());
						e.printStackTrace();
					} catch (RuntimeException e) {
						println("Caught RuntimeException and rethrowing: "
								+ e.getMessage());
						throw e;
					}

					// if (changeList.length > 1) {
					// System.out.println("done");
					// }

					// UPGRADE now gone
					// synchronized (m_gdp.m_componentStatus) {
					// m_gdp.m_componentStatus.m_changeQueue = 0;
					// }

					// unlock derived class now something has
					// been
					// done
					m_gdp.unlockComponent();

					if (used) {
						synchronized (m_gdp.m_wmChangeLock) {
							// now wake up any waiting threads
							m_gdp.m_wmChangeLock.notifyAll();
						}
					}
					// cout<<m_pWMRP->getComponentID()<<":
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
				} catch (InterruptedException e) {
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

			// UPGRADE now gone
			// synchronized (m_gdp.m_componentStatus) {
			// m_gdp.m_componentStatus.m_changeQueue++;
			// }

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
				} else if (m_gdp.m_queueBehaviour == WorkingMemoryChangeQueueBehaviour.QUEUE) {
					m_changes = new ConcurrentLinkedQueue<WorkingMemoryChange>();
					runQueue();
				}
			} catch (SubarchitectureComponentException e) {
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
	public WorkingMemoryReaderComponent() {
		receiveChanges();

		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		m_wmChangeRunnable = new WorkingMemoryChangeThread(this);
		m_wmChangeLock = new Object();

		m_changeObjects = new WorkingMemoryChangeFilterMap<WorkingMemoryChangeReceiver>();
		m_receiversToRemove = new Vector<WorkingMemoryChangeReceiver>();
	}

	// /**
	// * @param _filteredCount
	// * @param _totalCount
	// */
	// private void logChangeEvents(int _filteredCount, int _totalCount) {
	// synchronized (m_componentStatus) {
	// m_componentStatus.m_totalChangeEventsFiltered += _filteredCount;
	// m_componentStatus.m_totalChangeEventsReceived += _totalCount;
	// }
	// }
	//
	// private void logReadEvent(String _subarch, String _id) {
	// logEvent(ComponentEventType.GET, getComponentID(),
	// _subarch, "", _id);
	// }
	//
	// private void logReadEvent(String _subarch, String _type, int _count) {
	// logEvent(ComponentEventType.GET, getComponentID(),
	// _subarch, _type, "");
	// }

	protected int getFilterCount() {
		return m_changeObjects.size();
	}

	/**
	 * Remove a specific change receiver. If the list for this receiver is
	 * subsequently empty, remove filter.
	 * 
	 * @param _workingMemoryChangeReceiver
	 * @throws SubarchitectureComponentException
	 */
	private void removeChangeFilterHelper(
			WorkingMemoryChangeReceiver _workingMemoryChangeReceiver)
			throws SubarchitectureComponentException {

		Vector<WorkingMemoryChangeFilter> removed = new Vector<WorkingMemoryChangeFilter>();
		m_changeObjects.remove(_workingMemoryChangeReceiver, removed);

		for (WorkingMemoryChangeFilter filter : removed) {
			m_workingMemory.removeComponentFilter(filter);
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
	 * @throws SubarchitectureComponentException
	 */
	public void addChangeFilter(WorkingMemoryChangeFilter _filter,
			WorkingMemoryChangeReceiver _receiver) {

		assert (m_workingMemory != null);

		// sneaky operation to keep filter consistent without always
		// needing a component for info
		if (_filter.restriction == FilterRestriction.LOCALSA
				&& _filter.address.subarchitecture.length() == 0) {
			_filter.address.subarchitecture = getSubarchitectureID();
		}
		// also make sure subarch/restrictions are correct
		else if (_filter.address.subarchitecture.equals(getSubarchitectureID())
				&& _filter.restriction == FilterRestriction.ALLSA) {
			_filter.restriction = FilterRestriction.LOCALSA;
		}

		// and set origin
		_filter.origin = getComponentID();

		// if (existingReceiver == null) {

		m_changeObjects.put(_filter, _receiver);

		// println("new filters length: " + m_changeObjects.size());

		m_workingMemory.registerComponentFilter(_filter);
	}

	void getBaseWorkingMemoryEntries(String _type,
			List<WorkingMemoryEntry> _entries, String _subarch, int _count)
			throws UnknownSubarchitectureException {
		assert (_subarch.length() != 0);// subarch must not be empty
		assert (_type.length() != 0);
		assert (_entries != null);

		WorkingMemoryEntrySeqHolder holder = new WorkingMemoryEntrySeqHolder();
		m_workingMemory.getWorkingMemoryEntries(_type, _subarch, _count,
				getComponentID(), holder);

		// only update newly read things
		for (WorkingMemoryEntry entry : holder.value) {
			updateVersion(entry.id, entry.version);
			_entries.add(entry);
		}
	}

	void getBaseWorkingMemoryEntries(String _type,
			List<WorkingMemoryEntry> _entries, String _subarch)
			throws UnknownSubarchitectureException {
		getBaseWorkingMemoryEntries(_type, _entries, _subarch, 0);
	}

	void getBaseWorkingMemoryEntries(String _type,
			List<WorkingMemoryEntry> _entries, int _count) {
		try {
			getBaseWorkingMemoryEntries(_type, _entries,
					getSubarchitectureID(), _count);
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
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
	 * @throws SubarchitectureComponentException
	 *             Exception is thrown if communication or translation fails.
	 */
	public <Type extends Ice.Object> CASTData<Type>[] getWorkingMemoryEntries(
			Class<Type> _cls, int _count)
			throws SubarchitectureComponentException {
		return getWorkingMemoryEntries(getSubarchitectureID(), _cls, _count);
	}

	/**
	 * Retrieve the working memory entries matching the given type from the
	 * local subarchitecture working memory.
	 * 
	 * @param _cls
	 *            The class of the entries to be returned.
	 * @return An array of matching entries starting with the most recent.
	 * @throws SubarchitectureComponentException
	 *             Exception is thrown if communication or translation fails.
	 */
	public <Type extends Ice.Object> CASTData<Type>[] getWorkingMemoryEntries(
			Class<Type> _cls) throws SubarchitectureComponentException {
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
	 * @throws SubarchitectureComponentException
	 *             Exception is thrown if communication or translation fails.
	 */
	public <Type extends Ice.Object> CASTData<Type>[] getWorkingMemoryEntries(
			String _subarch, Class<Type> _cls)
			throws SubarchitectureComponentException {

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
	 * @throws UnknownSubarchitectureException
	 * @throws SubarchitectureComponentException
	 *             Exception is thrown if communication or translation fails.
	 */
	@SuppressWarnings("unchecked")
	@Deprecated
	public <Type extends Ice.Object> CASTData<Type>[] getWorkingMemoryEntries(
			String _subarch, Class<Type> _cls, int _count)
			throws UnknownSubarchitectureException {
		String type = CASTUtils.typeName(_cls);
		List<WorkingMemoryEntry> entries = new ArrayList<WorkingMemoryEntry>();
		getBaseWorkingMemoryEntries(type, entries, _subarch, _count);
		CASTData<Type>[] array = new CASTData[entries.size()];

		int i = 0;
		for (WorkingMemoryEntry entry : entries) {
			array[i++] = new CASTData<Type>(entry, _cls);
		}

		return array;
	}

	public <Type extends Ice.Object> void getMemoryEntries(Class<Type> _cls,
			List<Type> _entries, String _subarch, int _count)
			throws UnknownSubarchitectureException {
		String type = CASTUtils.typeName(_cls);
		List<WorkingMemoryEntry> entries = new ArrayList<WorkingMemoryEntry>();
		getBaseWorkingMemoryEntries(type, entries, _subarch, _count);
		for (WorkingMemoryEntry entry : entries) {
			_entries.add(_cls.cast(entry.entry));
		}
	}

	
	public <Type extends Ice.Object> void getMemoryEntries(Class<Type> _cls,
			List<Type> _entries)
			throws UnknownSubarchitectureException {
		getMemoryEntries(_cls, _entries, getSubarchitectureID(), 0);
	}
	
	public <Type extends Ice.Object> void getMemoryEntries(Class<Type> _cls,
			List<Type> _entries, String _subarch)
			throws UnknownSubarchitectureException {
		getMemoryEntries(_cls, _entries, _subarch, 0);
	}

	public <Type extends Ice.Object> void getMemoryEntries(Class<Type> _cls,
			List<Type> _entries, int _count) {
		try {
			getMemoryEntries(_cls, _entries, getSubarchitectureID(), _count);
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	public <Type extends Ice.Object> void getMemoryEntriesWithData(
			Class<Type> _cls, List<CASTData<Type>> _entries, String _subarch,
			int _count) throws UnknownSubarchitectureException {
		String type = CASTUtils.typeName(_cls);
		List<WorkingMemoryEntry> entries = new ArrayList<WorkingMemoryEntry>();
		getBaseWorkingMemoryEntries(type, entries, _subarch, _count);
		for (WorkingMemoryEntry entry : entries) {
			_entries.add(new CASTData<Type>(entry, _cls));
		}
	}

	public <Type extends Ice.Object> void getMemoryEntriesWithDataWithData(
			Class<Type> _cls, List<CASTData<Type>> _entries, String _subarch)
			throws UnknownSubarchitectureException {
		getMemoryEntriesWithData(_cls, _entries, _subarch, 0);
	}

	public <Type extends Ice.Object> void getMemoryEntriesWithData(
			Class<Type> _cls, List<CASTData<Type>> _entries, int _count) {
		try {
			getMemoryEntriesWithData(_cls, _entries, getSubarchitectureID(),
					_count);
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	WorkingMemoryEntry getBaseMemoryEntry(String _id)
			throws DoesNotExistOnWMException {
		try {
			return getBaseMemoryEntry(_id, getSubarchitectureID());
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	WorkingMemoryEntry getBaseMemoryEntry(WorkingMemoryAddress _wma)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		return getBaseMemoryEntry(_wma.id, _wma.subarchitecture);
	}

	WorkingMemoryEntry getBaseMemoryEntry(String _id, String _subarch)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		assert (_id.length() != 0);
		assert (_subarch.length() != 0);
		assert (m_workingMemory != null);
		WorkingMemoryEntry entry = m_workingMemory.getWorkingMemoryEntry(_id,
				_subarch, getComponentID());
		updateVersion(entry.id, entry.version);
		return entry;
	}

	/**
	 * Get the entry from working memory in a particular subarchitecture with
	 * the given id.
	 * 
	 * @param _id
	 *            The id for the entry in working memory.
	 * @param _subarch
	 *            The subarchitecture in which the id is located.
	 * @return The requested entry.
	 * @throws DoesNotExistOnWMException
	 */
	@Deprecated
	public CASTData<?> getWorkingMemoryEntry(String _id)
			throws DoesNotExistOnWMException {
		try {
			return getWorkingMemoryEntry(_id, getSubarchitectureID());
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	/**
	 * Get the entry from working memory in a particular subarchitecture with
	 * the given id.
	 * 
	 * @param _id
	 *            The id for the entry in working memory.
	 * @param _subarch
	 *            The subarchitecture in which the id is located.
	 * @return The requested entry.
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	@SuppressWarnings("unchecked")
	@Deprecated
	public CASTData<?> getWorkingMemoryEntry(String _id, String _subarch)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		WorkingMemoryEntry entry = getBaseMemoryEntry(_id, _subarch);
		return new CASTData(entry.id, entry.type, entry.version, entry.entry);
	}

	/**
	 * Get the entry from working memory in a particular subarchitecture with
	 * the given id.
	 * 
	 * @param _wma
	 *            The address of the thing to get
	 * @return The requested entry.
	 * @throws SubarchitectureComponentException
	 *             Exception is thrown if communication or translation fails.
	 */
	@Deprecated
	public CASTData<?> getWorkingMemoryEntry(WorkingMemoryAddress _wma)
			throws SubarchitectureComponentException {
		return getWorkingMemoryEntry(_wma.id, _wma.subarchitecture);
	}

	public <T extends Ice.Object> T getMemoryEntry(String _id, Class<T> _cls)
			throws DoesNotExistOnWMException {
		try {
			return getMemoryEntry(_id, getSubarchitectureID(), _cls);
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	public <T extends Ice.Object> T getMemoryEntry(String _id, String _subarch,
			Class<T> _cls) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		WorkingMemoryEntry entry = getBaseMemoryEntry(_id, _subarch);
		return _cls.cast(entry.entry);
	}

	public <T extends Ice.Object> T getMemoryEntry(WorkingMemoryAddress _wma,
			Class<T> _cls) throws DoesNotExistOnWMException,
			UnknownSubarchitectureException {
		return getMemoryEntry(_wma.id, _wma.subarchitecture, _cls);
	}

	public <T extends Ice.Object> CASTData<T> getMemoryEntryWithData(
			String _id, Class<T> _cls) throws DoesNotExistOnWMException {
		try {
			return getMemoryEntryWithData(_id, getSubarchitectureID(), _cls);
		} catch (UnknownSubarchitectureException e) {
			throw new RuntimeException(
					"Shouldn't happen on own subarchitecture", e);
		}
	}

	public <T extends Ice.Object> CASTData<T> getMemoryEntryWithData(
			String _id, String _subarch, Class<T> _cls)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		WorkingMemoryEntry entry = getBaseMemoryEntry(_id, _subarch);
		return new CASTData<T>(entry, _cls);
	}

	public <T extends Ice.Object> CASTData<T> getMemoryEntryWithData(
			WorkingMemoryAddress _wma, Class<T> _cls)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		return getMemoryEntryWithData(_wma.id, _wma.subarchitecture, _cls);
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
			throws SubarchitectureComponentException {
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
			} catch (InterruptedException e) {
				e.printStackTrace();
				System.exit(1);
			}
		}
	}

	/**
	 * Receives the WorkingMemoryChange list from a sub-architecture working
	 * memory and forwards this change to the underlying class in a new thread.
	 * Also locks access to the receiving thread whilst doing this.
	 * 
	 */

	public void receiveChangeEvent(WorkingMemoryChange _wmc, Current __current) {

		// no thread, and miss updates if blocking
		if (isRunning() && m_bReceivingChanges) {

			// prefer to use change objects
			if (m_changeObjects != null) {
				m_wmChangeRunnable.queueChange(_wmc);
			}
		}
	}

	/**
	 * Start this component running. This overridden method also starts the
	 * encapsulated thread that forwards change information.
	 * 
	 * @see balt.core.processes.FrameworkComponent#start()
	 */
	@Override
	public void startInternal() {
		super.startInternal();

		m_wmChangeRunnable.start();
		m_wmChangeThread = new Thread(m_wmChangeRunnable);

		m_wmChangeThread.setPriority(Thread.NORM_PRIORITY + 1);
		m_wmChangeThread.start();
	}

	/**
	 * Stop this component. This overridden method also stops the encapsulated
	 * thread that forwards change information.
	 * 
	 * @see balt.core.processes.FrameworkComponent#stop()
	 */
	@Override
	public void stopInternal() {
		super.stopInternal();

		// stop change thread
		m_wmChangeRunnable.stop();
		try {
			m_wmChangeThread.join();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		// wake threads waiting on changes
		synchronized (m_wmChangeLock) {
			m_wmChangeLock.notifyAll();
		}
	}

}
