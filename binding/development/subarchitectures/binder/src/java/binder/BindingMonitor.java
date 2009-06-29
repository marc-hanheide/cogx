package binder;

import binder.autogen.Proxy;
import cast.architecture.ManagedComponent;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;


public class BindingMonitor extends ManagedComponent {


	@Override
	public void start() {
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Proxy.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
				Proxy newProxy = getMemoryEntry(_wmc.address, Proxy.class);
				handleNewProxy(newProxy);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}
	
	
	public void handleNewProxy(Proxy newProxy) {
		log("Proxy successfully received in the binding monitor!");
	}
}
