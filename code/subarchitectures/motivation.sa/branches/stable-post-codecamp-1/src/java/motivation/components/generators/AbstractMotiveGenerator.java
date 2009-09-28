/**
 *
 * 
 */
package motivation.components.generators;

import motivation.slice.Motive;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;

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
		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			if (_wmc.operation == WorkingMemoryOperation.OVERWRITE)
				overwritten(_wmc);
			else if (_wmc.operation == WorkingMemoryOperation.DELETE)
				deleted(_wmc);
		}

		public void overwritten(WorkingMemoryChange _wmc) {
			log("source has changed, need to update motive");
			try {
				lockEntry(motiveAddress, WorkingMemoryPermissions.LOCKEDO);
				Motive motive = getMemoryEntry(motiveAddress, Motive.class);
				checkMotive(motive);
				unlockEntry(motiveAddress);
			} catch (DoesNotExistOnWMException e) {
				println("trying to lock non-existing object. the motive seems to be gone... just ignore, considered a CAST bug.");
				//e.printStackTrace();
			} catch (UnknownSubarchitectureException e) {
				println("subarchitecture " + motiveAddress.subarchitecture
						+ " not known...");
				e.printStackTrace();
			} catch (ConsistencyException e) {
				println("ConsistencyException in Generator::overwritter");
				e.printStackTrace();
			}
		}

		public void deleted(WorkingMemoryChange _wmc) {
			try {
				println("source has been removed, need to remove motive");
				Motive motive = getMemoryEntry(motiveAddress, Motive.class);
				remove(motive);
				removeChangeFilter(this);
			} catch (DoesNotExistOnWMException e1) {
				println("tried to remove motive from WM that didn't exist... nevermind.");
				e1.printStackTrace();
			} catch (PermissionException e1) {
				println("not allowed to remove motive from WM...  nevermind.");
				e1.printStackTrace();
			} catch (UnknownSubarchitectureException e1) {
				println("UnknownSubarchitectureException when remove motive from WM...  nevermind.");
				e1.printStackTrace();
			} catch (SubarchitectureComponentException e) {
				println("exception removing WorkingMemoryChangeReceiver: "
						+ e.message);
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}

	protected abstract boolean checkMotive(Motive motive);

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
		log("add receivers for this motive to link it to its source");
		new SourceChangeReceiver(motive, src);
	}

	/**
	 * write to the WM, either create a new entry (if the motive was not yet in
	 * their) or update the existing one
	 * 
	 * @param motive
	 * @return the WM address of the entry
	 */
	public WorkingMemoryAddress write(Motive motive) {
		motive.updated = getCASTTime();
		try {
		if (motive.thisEntry == null) {
			log("submit new to WM");
			motive.thisEntry = new WorkingMemoryAddress();
			motive.thisEntry.subarchitecture = getSubarchitectureID();
			motive.thisEntry.id = newDataID();
			addToWorkingMemory(motive.thisEntry, motive);
			log("added " + motive.thisEntry.subarchitecture + "::"+motive.thisEntry.id);
			addReceivers(motive.thisEntry, motive.referenceEntry);
			log("receivers added");
		} else {
			overwriteWorkingMemory(motive.thisEntry, motive);
		}
		}
		catch(PermissionException e) {
			
		} catch (CASTException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return motive.thisEntry;
	}

	public void remove(Motive motive) throws DoesNotExistOnWMException,
			PermissionException, UnknownSubarchitectureException {
		if (motive.thisEntry != null) {
			log("we remove the motive from WM");
			deleteFromWorkingMemory(motive.thisEntry);
		}
	}
}
