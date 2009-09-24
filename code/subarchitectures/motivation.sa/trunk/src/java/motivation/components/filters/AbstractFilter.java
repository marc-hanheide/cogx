/**
 * 
 */
package motivation.components.filters;

import java.util.LinkedList;
import java.util.List;

import motivation.slice.CategorizePlaceMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;

/**
 * @author marc
 * 
 */
public abstract class AbstractFilter extends ManagedComponent {

	WorkingMemoryChangeReceiver receiver;

	protected AbstractFilter() {
		super();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		// TODO Auto-generated method stub
		super.start();

		receiver = new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				// avoid self calls
				if (_wmc.src.equals(getComponentID()))
					return;
				log("src of motive change: " + _wmc.src);
				try {
					lockEntry(_wmc.address, WorkingMemoryPermissions.LOCKEDO);
					Motive motive = getMemoryEntry(_wmc.address, Motive.class);
					switch (motive.status) {
					case UNSURFACED:
						log("check unsurfaced motive " + motive.toString());
						if (shouldBeSurfaced(motive)) {
							log("-> surfaced motive " + motive.toString());
							motive.status = MotiveStatus.SURFACED;
							overwriteWorkingMemory(_wmc.address, motive);
						}
						break;
					case ACTIVE:
					case SURFACED:
						log("check surfaced motive " + motive.toString());
						if (shouldBeUnsurfaced(motive)) {
							log("-> unsurfaced motive " + motive.toString());
							motive.status = MotiveStatus.UNSURFACED;
							overwriteWorkingMemory(_wmc.address, motive);
						}
						break;
					}
				} catch (DoesNotExistOnWMException e) {
					println("filter failed to access motive from WM, maybe has been removed... ignore");
				} catch (UnknownSubarchitectureException e) {
					println("UnknownSubarchitectureException: this shouldn't happen.");
					e.printStackTrace();
				} catch (ConsistencyException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (PermissionException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} finally {
					try {
						unlockEntry(_wmc.address);
					} catch (ConsistencyException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} catch (DoesNotExistOnWMException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					} catch (UnknownSubarchitectureException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}

			}
		};

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Motive.class,
				WorkingMemoryOperation.ADD), receiver);
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Motive.class,
				WorkingMemoryOperation.OVERWRITE), receiver);

	}

	protected void checkAll() throws CASTException {
		// TODO: currently CAST::getMemoryEntries only considers the "real" type
		// of entries, whihc means, getMemoryEntries(Motive.class) will not
		// work... is a feature request for CAST
		List<ExploreMotive> exploreMotives;
		exploreMotives = new LinkedList<ExploreMotive>();
		getMemoryEntries(ExploreMotive.class, exploreMotives);
		// trigger them all by overwriting them
		for (Motive m : exploreMotives) {
			WorkingMemoryChange wmc = new WorkingMemoryChange();
			wmc.address = m.thisEntry;
			wmc.operation = WorkingMemoryOperation.OVERWRITE;
			wmc.src = "explicit self-trigger";
			receiver.workingMemoryChanged(wmc);
		}

		List<HomingMotive> homingMotives;
		homingMotives = new LinkedList<HomingMotive>();
		getMemoryEntries(HomingMotive.class, homingMotives);
		// trigger them all by overwriting them
		for (Motive m : homingMotives) {
			WorkingMemoryChange wmc = new WorkingMemoryChange();
			wmc.address = m.thisEntry;
			wmc.operation = WorkingMemoryOperation.OVERWRITE;
			wmc.src = "explicit self-trigger";
			receiver.workingMemoryChanged(wmc);
		}
	
		List<CategorizePlaceMotive> categorizePlaceMotives;
		categorizePlaceMotives = new LinkedList<CategorizePlaceMotive>();
		getMemoryEntries(CategorizePlaceMotive.class, categorizePlaceMotives);
		// trigger them all by overwriting them
		for (Motive m : categorizePlaceMotives) {
			WorkingMemoryChange wmc = new WorkingMemoryChange();
			wmc.address = m.thisEntry;
			wmc.operation = WorkingMemoryOperation.OVERWRITE;
			wmc.src = "explicit self-trigger";
			receiver.workingMemoryChanged(wmc);
		}
	
	
	}

	/**
	 * @param motive
	 * @return
	 */
	protected abstract boolean shouldBeUnsurfaced(Motive motive);

	/**
	 * @param motive
	 * @return
	 */
	protected abstract boolean shouldBeSurfaced(Motive motive);

}
