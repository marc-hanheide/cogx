/**
 *
 * 
 */
package motivation.components.generators;

import java.util.concurrent.LinkedBlockingQueue;

import motivation.slice.Motive;
import motivation.util.WMMotiveView;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
/**
 * @author marc
 *
 */
/**
 * @author marc
 * 
 */
public abstract class AbstractMotiveGenerator extends ManagedComponent {

	protected WMMotiveView motives;

	/**
	 * a local MemoryReceiver that established the link between the "Supporter"
	 * of the motive and
	 * 
	 * @author marc
	 * 
	 */
	public class SourceChangeReceiver implements WorkingMemoryChangeReceiver {
		/**
		 * @param motiveAddress
		 */
		public SourceChangeReceiver(WorkingMemoryAddress motiveAddress,
				WorkingMemoryAddress reference) {
			super();
			this.motiveAddress = motiveAddress;
			addChangeFilter(ChangeFilterFactory.createAddressFilter(reference,
					WorkingMemoryOperation.OVERWRITE), this);
			addChangeFilter(ChangeFilterFactory.createAddressFilter(reference,
					WorkingMemoryOperation.DELETE), this);
		}

		WorkingMemoryAddress motiveAddress;

		/*
		 * (non-Javadoc)
		 * 
		 * @see
		 * cast.architecture.WorkingMemoryChangeReceiver#workingMemoryChanged
		 * (cast.cdl.WorkingMemoryChange)
		 */
		@Override
		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			log("one reference with id " + _wmc.address.subarchitecture + "::"
					+ _wmc.address.id
					+ " has changed, need to update motive with OP "
					+ _wmc.operation.name());
			if (_wmc.operation == WorkingMemoryOperation.OVERWRITE)
				overwritten(_wmc);
			else if (_wmc.operation == WorkingMemoryOperation.DELETE)
				deleted(_wmc);
		}

		public void overwritten(WorkingMemoryChange _wmc) {
			try {
				// lockEntry(motiveAddress, WorkingMemoryPermissions.LOCKEDODR);
				Motive motive = getMemoryEntry(motiveAddress, Motive.class);
				scheduleCheckMotive(motive);
			} catch (DoesNotExistOnWMException e) {
				println("trying to lock non-existing object. the motive seems to be gone... just ignore, considered a CAST bug.");
				// e.printStackTrace();
			} catch (UnknownSubarchitectureException e) {
				println("subarchitecture " + motiveAddress.subarchitecture
						+ " not known...");
				e.printStackTrace();
				// } finally {
				// try {
				// unlockEntry(motiveAddress);
				// } catch (CASTException e) {
				// log("caught a CASTException when unlocking entry: "
				// + e.message + ". This can be safely ignored...");
				// }
			}
		}

		public void deleted(WorkingMemoryChange _wmc) {
			try {
				// lockEntry(motiveAddress, WorkingMemoryPermissions.LOCKEDODR);
				Motive motive = getMemoryEntry(motiveAddress, Motive.class);
				if (motive.referenceEntry.equals(_wmc.address)) { // if it is
					// really
					// the
					// source
					// that is
					// deleted
					log("source " + _wmc.address.subarchitecture + "::"
							+ motive.referenceEntry.id + "/"
							+ _wmc.address.subarchitecture
							+ "has been removed, need to remove motive");
					remove(motive);
					removeChangeFilter(this);

				} else { // if it is some other link entry, do not delete, but
					// check
					log("referenced entry " + _wmc.address.subarchitecture
							+ "::" + motive.referenceEntry.id + "/"
							+ _wmc.address.subarchitecture
							+ " has been deleted, need to check motive");
					scheduleCheckMotive(motive);
				}
			} catch (DoesNotExistOnWMException e1) {
				println("tried to remove motive from WM that didn't exist... nevermind.");
			} catch (CASTException e1) {
				println("CASTException...  nevermind.");
				e1.printStackTrace();
				// } catch (InterruptedException e) {
				// // TODO Auto-generated catch block
				// e.printStackTrace();
				// } finally {
				// try {
				// unlockEntry(motiveAddress);
				// } catch (CASTException e) {
				// log("caught a CASTException when unlocking entry: "
				// + e.message + ". This can be safely ignored...");
				// }
			}

		}
	}

	LinkedBlockingQueue<Motive> checkMotiveQueue;

	/**
	 * @param checkMotiveQueue
	 */
	public AbstractMotiveGenerator() {
		super();
		this.checkMotiveQueue = new LinkedBlockingQueue<Motive>();
		this.motives = WMMotiveView.create(this);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		try {
			motives.start();
		} catch (UnknownSubarchitectureException e) {
			logException("couldn't start view", e);
		}
	}

	protected abstract boolean checkMotive(Motive motive) throws CASTException;

	/**
	 * creates a link between the underlying referenced working memory entry
	 * that created the motive and the motive itself. Assures, that any
	 * modification of the entry that gave rise to the motive is propagated to
	 * the motive object.
	 * 
	 * @param motive
	 * @param src
	 */
	protected void addReceivers(WorkingMemoryAddress motive,
			WorkingMemoryAddress src) {
		log("add receivers for this motive " + CASTUtils.toString(motive)
				+ " to link it to its source " + CASTUtils.toString(src));
		new SourceChangeReceiver(motive, src);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected final void runComponent() {
		while (isRunning()) {
			Motive motive = null;
			try {
				log("runComponent: waiting for motive event");
				motive = checkMotiveQueue.take();
				if (motive.thisEntry != null) {
					// update from WM
					try {
						motive = getMemoryEntry(motive.thisEntry, Motive.class);
					} catch (CASTException e) {
						// ignore this exception
						println("CASTExecption in runComponent while reading the motive from WM... shouldn't be a big deal: "
								+ e.message);
					}

				}
				log("runComponent: check motive");
				checkMotive(motive);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (CASTException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	/**
	 * write to the WM, either create a new entry (if the motive was not yet in
	 * their) or update the existing one
	 * 
	 * @param motive
	 * @return the WM address of the entry
	 * @throws CASTException
	 */
	public WorkingMemoryAddress write(Motive motive) throws CASTException {
		motive.updated = getCASTTime();
		if (motive.thisEntry == null) {
			log("submit new to WM");
			motive.thisEntry = new WorkingMemoryAddress();
			motive.thisEntry.subarchitecture = getSubarchitectureID();
			motive.thisEntry.id = newDataID();
			log("added " + motive.thisEntry.subarchitecture + "::"
					+ motive.thisEntry.id);
			addReceivers(motive.thisEntry, motive.referenceEntry);
			log("receivers added");
			addToWorkingMemory(motive.thisEntry, motive);
		} else {
			overwriteWorkingMemory(motive.thisEntry, motive);
		}
		return motive.thisEntry;
	}

	public void remove(Motive motive) throws CASTException {
		if (motive.thisEntry != null) {
			log("we remove the motive from WM with ID " + motive.thisEntry.id);
			try {
				deleteFromWorkingMemory(motive.thisEntry);
			} catch (DoesNotExistOnWMException e) {
				// safely ignore if it has been deleted already
			}
		}
	}

	protected void scheduleCheckMotive(Motive motive) {
		try {
			checkMotiveQueue.put(motive);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
