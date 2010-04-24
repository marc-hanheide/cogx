/**
 * 
 */
package motivation.components.managers;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveSet;
import Ice.ObjectImpl;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEntrySet;

/**
 * @author marc
 * 
 */
public abstract class AbstractMotiveManager extends ManagedComponent {

	Class<? extends Motive> specificType;
	WMMotiveSet motives;
	WMEntrySet placeOrigins;

	/**
	 * @param specificType
	 */
	protected AbstractMotiveManager(Class<? extends Motive> specificType) {
		super();
		this.specificType = specificType;
		motives = WMMotiveSet.create(this, specificType);
//		placeOrigins = WMEntrySet.create(this, OriginMap.class);
	}

//	/**
//	 * asks the spatial.sa for the lookup map to get from Places to binder
//	 * proxies
//	 * 
//	 * @return the map or null if one couldn't be retrieved from the spatial WM
//	 */
//	OriginMap getOriginMap() {
//		List<OriginMap> l = new LinkedList<OriginMap>();
//		try {
//			getMemoryEntries(OriginMap.class, l, "spatial.sa");
//		} catch (CASTException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//		if (l.size() == 0) {
//			return null;
//		}
//		for (String s : l.get(0).sourceID2ProxyID.values()) {
//			log("  originMap: " + s);
//		}
//		return l.get(0);
//	}

	@Override
	protected void start() {
		log("MotiveManager starting up...");
		super.start();

		WMEntrySet.ChangeHandler manageHandler = new WMEntrySet.ChangeHandler() {
			@Override
			public void entryChanged(
					Map<WorkingMemoryAddress, ObjectImpl> map,
					WorkingMemoryChange wmc, ObjectImpl newMotive,
					ObjectImpl oldMotive) {
				log("new Motive ");
				manageMotive((Motive) newMotive);
			}
		};

		WMEntrySet.ChangeHandler retractHandler = new WMEntrySet.ChangeHandler() {
			@Override
			public void entryChanged(
					Map<WorkingMemoryAddress, ObjectImpl> map,
					WorkingMemoryChange wmc, ObjectImpl newMotive,
					ObjectImpl oldMotive) {
				if (newMotive != null
						&& ((Motive) newMotive).status != MotiveStatus.ACTIVE) {
					log("retract Motive triggered");
					retractMotive((Motive) newMotive);
				}
			}
		};

		// transition from unsurfaced to surfaced triggers a motive to be managed
		motives.setStateChangeHandler(new WMMotiveSet.MotiveStateTransition(
				MotiveStatus.UNSURFACED, MotiveStatus.SURFACED), manageHandler);

		// transition from surfaced to anything (besides ACTIVE) triggers a motive to be unmanaged
		motives.setStateChangeHandler(new WMMotiveSet.MotiveStateTransition(
				MotiveStatus.SURFACED, null), retractHandler);

		// transition from surfaced to anything (besides ACTIVE) triggers a motive to be unmanaged
		motives.setStateChangeHandler(new WMMotiveSet.MotiveStateTransition(
				MotiveStatus.ACTIVE, null), retractHandler);
		


		// motives.setHandler(new WMMotiveSet.ChangeHandler() {
		// @Override
		// public void motiveChanged(
		// Map<WorkingMemoryAddress, Ice.ObjectImpl> map,
		// WorkingMemoryChange wmc, Ice.ObjectImpl o, Ice.ObjectImpl old) {
		// Motive motive = (Motive) o;
		// log("motive has been changed in map: Status is "
		// + motive.status.name());
		// if ((wmc.operation == WorkingMemoryOperation.ADD)
		// || (wmc.operation == WorkingMemoryOperation.OVERWRITE)) {
		// switch (motive.status) {
		// case UNSURFACED:
		// retractMotive(motive);
		// break;
		// case SURFACED:
		// manageMotive(motive);
		// break;
		// }
		// } else if (wmc.operation == WorkingMemoryOperation.DELETE) {
		// retractMotive(motive);
		// }
		// }
		//
		// });

		// start the motive listener...
		motives.start();
	}

	protected abstract void activateMotive(Motive newMotive);

	protected abstract void manageMotive(Motive motive);

	protected abstract void retractMotive(Motive motive);
}
