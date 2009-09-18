/**
 * 
 */
package binder.components;

import binder.autogen.core.OriginMap;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;

/**
 * Just a simple class to print out the origins of proxies.
 * 
 * @author nah
 * 
 */
public class OriginListener extends ManagedComponent {

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory
				.createGlobalTypeFilter(OriginMap.class),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc)
							throws CASTException {
						displayOrigins(getMemoryEntry(_wmc.address,
								OriginMap.class), _wmc.address.subarchitecture);
					}

				});
	}

	private void displayOrigins(OriginMap _memoryEntry, String _subarch)
			throws UnknownSubarchitectureException {
		println("Component " + _memoryEntry.componentID
				+ " created the following proxies:");
		for (String localID : _memoryEntry.sourceID2ProxyID.keySet()) {
			String proxyID = _memoryEntry.sourceID2ProxyID.get(localID);
			println(CASTUtils.concatenate(localID, " (exists=",
					existsOnWorkingMemory(localID, _subarch), ") -> ", proxyID,
					" (exists=", existsOnWorkingMemory(proxyID, _subarch), ")"));
		}
	}
}
