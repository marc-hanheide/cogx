/**
 * 
 */
package motivation.components.managers;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import motivation.slice.Motive;
import motivation.util.WMEntrySet;
import motivation.util.WMMotiveSet;
import binder.autogen.core.OriginMap;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public abstract class MotiveManager extends ManagedComponent {

	Class<? extends Motive> specificType;
	WMMotiveSet motives;
	WMEntrySet placeOrigins;
	
	/**
	 * @param specificType
	 */
	protected MotiveManager(Class<? extends Motive> specificType) {
		super();
		this.specificType = specificType;
		motives = WMMotiveSet.create(this, specificType);
		placeOrigins = WMEntrySet.create(this,OriginMap.class);
	}
	
	/** asks the spatial.sa for the lookup map to get from Places to binder proxies

	 * @return the map or null if one couldn't be retrieved from the spatial WM
	 */
	OriginMap getOriginMap() {
		List<OriginMap> l = new LinkedList<OriginMap>();
		try {
			getMemoryEntries(OriginMap.class, l,"spatial.sa");
		} catch (CASTException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		if (l.size()==0) {
			return null;
		}
		for (String s : l.get(0).sourceID2ProxyID.values()) {
			log("  originMap: "+s);
		}
		return l.get(0);
	}

	@Override
	protected void start() {
		log("MotiveManager starting up...");
		super.start();

		motives.setHandler(new WMMotiveSet.ChangeHandler() {

			@Override
			public void motiveChanged(
					Map<WorkingMemoryAddress, Ice.ObjectImpl> map,
					WorkingMemoryChange wmc, Ice.ObjectImpl o) {
				Motive motive = (Motive) o;
				log("motive has been changed in map: Status is "
						+ motive.status.name());
				if ((wmc.operation == WorkingMemoryOperation.ADD)
						|| (wmc.operation == WorkingMemoryOperation.OVERWRITE)) {
					switch (motive.status) {
					case UNSURFACED:
						retractMotive(motive);
						break;
					case SURFACED:
						manageMotive(motive);
						break;
					}
				} else if (wmc.operation == WorkingMemoryOperation.DELETE) {
					retractMotive(motive);
				}
			}

		});

		// start the motive listener...
		motives.start();
	}

	protected abstract void manageMotive(Motive motive);

	protected abstract void retractMotive(Motive motive);
}
