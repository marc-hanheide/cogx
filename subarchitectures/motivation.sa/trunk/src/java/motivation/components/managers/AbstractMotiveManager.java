/**
 * 
 */
package motivation.components.managers;

import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveView;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEntrySet;
import castutils.castextensions.WMView;

/**
 * @author marc
 * 
 */
public abstract class AbstractMotiveManager extends ManagedComponent {

	WMMotiveView motives;
	WMEntrySet placeOrigins;

	/**
	 * @param specificType
	 */
	protected AbstractMotiveManager() {
		super();

		motives = WMMotiveView.create(this);
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

		WMView.ChangeHandler<Motive> manageHandler = new WMView.ChangeHandler<Motive>() {
			@Override
			public void entryChanged(
					Map<WorkingMemoryAddress, Motive> map,
					WorkingMemoryChange wmc, Motive newMotive,
					Motive oldMotive) {
				log("new Motive ");
				manageMotive(newMotive);
			}
		};

		WMView.ChangeHandler<Motive> retractHandler = new WMView.ChangeHandler<Motive>() {
			@Override
			public void entryChanged(
					Map<WorkingMemoryAddress, Motive> map,
					WorkingMemoryChange wmc, Motive newMotive,
					Motive oldMotive) {
				if (newMotive != null
						&& (newMotive).status != MotiveStatus.ACTIVE) {
					log("retract Motive triggered");
					retractMotive(newMotive);
				}
			}
		};

		// transition from unsurfaced to surfaced triggers a motive to be managed
		motives.setStateChangeHandler(new WMMotiveView.MotiveStateTransition(
				MotiveStatus.UNSURFACED, MotiveStatus.SURFACED), manageHandler);

		// transition from surfaced to anything (besides ACTIVE) triggers a motive to be unmanaged
		motives.setStateChangeHandler(new WMMotiveView.MotiveStateTransition(
				MotiveStatus.SURFACED, null), retractHandler);

		// transition from surfaced to anything (besides ACTIVE) triggers a motive to be unmanaged
		motives.setStateChangeHandler(new WMMotiveView.MotiveStateTransition(
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
		try {
			motives.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
	}

	protected abstract void activateMotive(Motive newMotive);

	protected abstract void manageMotive(Motive motive);

	protected abstract void retractMotive(Motive motive);
}
