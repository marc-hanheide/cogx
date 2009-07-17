package binder.components;

import java.util.Enumeration;
import java.util.Vector;

import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.gui.BinderMonitorGUI;
import cast.architecture.ManagedComponent;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


public class BinderMonitor extends ManagedComponent {

	BinderMonitorGUI gui;
	
	Vector<Proxy> lastProxies;
	Vector<Union> lastUnions;
	
	@Override
	public void start() {
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Proxy.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
	//			Proxy newProxy = getMemoryEntry(_wmc.address, Proxy.class);
				updateMonitor();
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Union.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
	//			Union newUnion = getMemoryEntry(_wmc.address, Union.class);
				updateMonitor();
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		lastProxies = new Vector<Proxy>();
		lastUnions = new Vector<Union>();
		
		 gui = new BinderMonitorGUI(this);
		 gui.LOGGING = m_bLogOutput;
		log("Binding Monitor successfully started");
	}
	
	 
	public void updateMonitor() {
		log("Change in the binding working memory, update necessary");
		Vector<Proxy> proxiesV = new Vector<Proxy>();
		Vector<Union> unionsV = new Vector<Union>();
		
		Vector<Proxy> proxiesToDelete = new Vector<Proxy>();
		Vector<Union> unionsToDelete= new Vector<Union>();
		
		try {
			CASTData<Proxy>[] proxies = getWorkingMemoryEntries(Proxy.class);
			for (int i = (proxies.length - 1) ; i >= 0 ; i--) {
				proxiesV.add(proxies[i].getData());
			}
			CASTData<Union>[] unions = getWorkingMemoryEntries(Union.class);
			for (int i = 0 ; i < unions.length ; i++) {
				unionsV.add(unions[i].getData());
			}
						
			for (Enumeration<Proxy> e = lastProxies.elements(); e.hasMoreElements(); ) {
				Proxy proxy = e.nextElement();
				if (!proxiesV.contains(proxy)) {
					proxiesToDelete.add(proxy);
				}
			}
			for (Enumeration<Union> e = lastUnions.elements(); e.hasMoreElements(); ) {
				Union union = e.nextElement();
				if (!unionsV.contains(union)) {
					unionsToDelete.add(union);
				}
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		lastProxies = proxiesV;
		lastUnions = unionsV;
		
		log("number of proxies: " + proxiesV.size());
		log("number of unions: " + unionsV.size());
		gui.updateGUI(proxiesV, unionsV, proxiesToDelete, unionsToDelete);
	}
	
	
}
