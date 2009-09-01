package binder.abstr;

import java.util.Vector;

import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.utils.GradientDescent;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

/**
 * Abstract class for retrieving elements currently present in the working memory
 * of the binder
 * 
 * @author Pierre Lison
 * @version 31/08/2009
 */
public class BindingWorkingMemoryReader extends ManagedComponent {

	
	
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
					log("YOOOHOO!!!");
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
	}

	
}
