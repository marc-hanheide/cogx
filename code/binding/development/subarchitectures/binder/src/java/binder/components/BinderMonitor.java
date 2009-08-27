package binder.components;

import java.util.Enumeration;
import java.util.Map;
import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.core.UnionDistribution;
import binder.gui.BinderMonitorGUI;
import binder.utils.GradientDescent;
import binder.utils.ProbDistribUtils;
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
		//		updateMonitor();
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(AlternativeUnionConfigurations.class,
				WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
				AlternativeUnionConfigurations config = getMemoryEntry(_wmc.address, AlternativeUnionConfigurations.class);
				updateMonitor(config);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		lastProxies = new Vector<Proxy>();
		lastUnions = new Vector<Union>();
		
		log("Binding Monitor successfully started");
	}
	
	

	@Override
	public void configure(Map<String, String> _config) {
		if (_config.containsKey("--gui")) {
			 gui = new BinderMonitorGUI(this);
	//		 gui.LOGGING = false;
		} 
	}
	
	
	public void updateMonitor(AlternativeUnionConfigurations config) {
		log("Change in the binding working memory, update necessary");
		Vector<Proxy> proxiesV = new Vector<Proxy>();
		Vector<Union> UnionsV = new Vector<Union>();
		
		Vector<Proxy> proxiesToDelete = new Vector<Proxy>();
		Vector<Union> UnionsToDelete= new Vector<Union>();
		
		try {
			CASTData<Proxy>[] proxies = getWorkingMemoryEntries(Proxy.class);
			log("Current number of proxies: " + proxies.length);
			for (int i = (proxies.length - 1) ; i >= 0 ; i--) {
				proxiesV.add(proxies[i].getData());
			}
			
			UnionConfiguration bestConfig = GradientDescent.getBestUnionConfiguration(config);
			

			for (int i = 0 ; i < bestConfig.includedUnions.length ; i++) {
				UnionsV.add(bestConfig.includedUnions[i]);
				}
			
						
			for (Enumeration<Proxy> e = lastProxies.elements(); e.hasMoreElements(); ) {
				Proxy proxy = e.nextElement();
				if (!proxiesV.contains(proxy)) {
					proxiesToDelete.add(proxy);
				}
			}
			for (Enumeration<Union> e = lastUnions.elements(); e.hasMoreElements(); ) {
				Union union = e.nextElement();
				if (!UnionsV.contains(union)) {
					UnionsToDelete.add(union);
				}
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		
		lastProxies = proxiesV;
		lastUnions = UnionsV;
		log("Current number of Unions: " + UnionsV.size());
		
		if (gui != null) {
			gui.updateGUI(proxiesV, UnionsV, proxiesToDelete, UnionsToDelete);
		}
	}
	
	
}
