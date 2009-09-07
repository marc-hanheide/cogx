/**
 *
 * 
 */
package motivation.generators;

import motivation.slice.Motive;
import cast.AlreadyExistsOnWMException;
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
import cast.core.CASTUtils;
import cast.interfaces.TimeServerPrx;

/**
 * @author marc
 * 
 */
abstract class Generator extends ManagedComponent {

	TimeServerPrx timeServer;

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
			updateMotive(motiveAddress, _wmc.address);
		}

		public void deleted(WorkingMemoryChange _wmc) {
			try {
				log("source has been removed, need to remove motive");
				deleteFromWorkingMemory(motiveAddress);
			} catch (DoesNotExistOnWMException e1) {
				log("tried to remove motive from WM that didn't exist... nevermind.");
				e1.printStackTrace();
			} catch (PermissionException e1) {
				log("not allowed to remove motive from WM...  nevermind.");
				e1.printStackTrace();
			} catch (UnknownSubarchitectureException e1) {
				log("UnknownSubarchitectureException when remove motive from WM...  nevermind.");
				e1.printStackTrace();
			}
			try {
				removeChangeFilter(this);
			} catch (SubarchitectureComponentException e) {
				log("exception removing WorkingMemoryChangeReceiver: "
						+ e.message);
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}

	/**
	 * 
	 */
	public Generator() {
		// TODO Auto-generated constructor stub
		timeServer = CASTUtils.getTimeServer();
	}

	protected abstract boolean updateMotive(WorkingMemoryAddress motiveAddress,
			WorkingMemoryAddress srcAddress);

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
		debug("add receivers for this motive to link it to its source");
		new SourceChangeReceiver(motive, src);
	}

	public void overwrite(WorkingMemoryAddress motiveAddress, Motive motive)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException,
			ConsistencyException, PermissionException {
		motive.updated = timeServer.getCASTTime();
		log("overwrite it now");
		overwriteWorkingMemory(motiveAddress, motive);

	}

	public WorkingMemoryAddress add(Motive motive)
			throws AlreadyExistsOnWMException {
		WorkingMemoryAddress newMotiveAddress = new WorkingMemoryAddress();
		newMotiveAddress.id = newDataID();
		newMotiveAddress.subarchitecture = getSubarchitectureID();
		addToWorkingMemory(newMotiveAddress.id, motive);
		addReceivers(newMotiveAddress, motive.referenceEntry);
		updateMotive(newMotiveAddress, motive.referenceEntry);
		return newMotiveAddress;
	}
}
