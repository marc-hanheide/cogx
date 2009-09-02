package binder.abstr;


import java.util.Vector;

import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * Abstract class for retrieving elements currently present in the working memory
 * of the binder
 * 
 * @author Pierre Lison
 * @version 31/08/2009
 */
public class BindingWorkingMemoryReader extends ManagedComponent {

	
	Vector<Union> currentUnions ;
	
	@Override
	public void start() {

		// if the set of possible union configurations has been updated, update the
		// monitor accordingly
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					UnionConfiguration config = 
						getMemoryEntry(_wmc.address, UnionConfiguration.class);
					extractUnionsFromConfig(config);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		currentUnions = new Vector<Union>();
		
	}

	
	private void extractUnionsFromConfig (UnionConfiguration config) {
		currentUnions = new Vector<Union>();
		for (int i = 0; i < config.includedUnions.length ; i++) {
			currentUnions.add(config.includedUnions[i]);
		}
	}
	
	public Vector<Union> getUnions () {
		return currentUnions;
	}
	
}
