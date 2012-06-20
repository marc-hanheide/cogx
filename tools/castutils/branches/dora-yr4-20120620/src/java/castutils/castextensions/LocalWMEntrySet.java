/**
 * 
 */
package castutils.castextensions;

import Ice.ObjectImpl;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 *
 */
public class LocalWMEntrySet extends WMEntrySet {

	protected LocalWMEntrySet(ManagedComponent c) {
		super(c);
		// TODO Auto-generated constructor stub
	}

	/* (non-Javadoc)
	 * @see castutils.WMEntrySet#register(java.lang.Class)
	 */
	@Override
	public void register(Class<? extends ObjectImpl> type) {
		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(type,
				WorkingMemoryOperation.ADD), new WMChangeReceiver(type));
		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(type,
				WorkingMemoryOperation.DELETE), new WMChangeReceiver(type));
		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(type,
				WorkingMemoryOperation.OVERWRITE), new WMChangeReceiver(type));

	}

	

}
