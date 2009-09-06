/**
 * 
 */
package motivation.util;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import motivation.slice.Motive;

import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class WMMotiveSet {
	/**
	 * 
	 */
	private static final long serialVersionUID = 6388467413187493228L;

	private ManagedComponent component;

	private Map<WorkingMemoryAddress, Motive> map;


	/** Factory method 
	 * @param c he management component this WMSet is in
	 */
	public static WMMotiveSet create(ManagedComponent c) {
		return new WMMotiveSet(c);
	}
	
	protected WMMotiveSet(ManagedComponent c) {
		this.component = c;
		// create a synchronised hashmap
		map = Collections
				.synchronizedMap(new HashMap<WorkingMemoryAddress, Motive>());
		// register the addition
		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				Motive.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							map.put(_wmc.address, component.getMemoryEntry(
									_wmc.address, Motive.class));
						} catch (DoesNotExistOnWMException e) {
							// safely ignored if it is already gone
						} catch (UnknownSubarchitectureException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				});

		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				Motive.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						map.remove(_wmc.address);
					}
				});

		component.addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				Motive.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							map.put(_wmc.address, component.getMemoryEntry(
									_wmc.address, Motive.class));
						} catch (DoesNotExistOnWMException e) {
							// remove it locally
							map.remove(_wmc.address);
						} catch (UnknownSubarchitectureException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					}
				});

	}
}
