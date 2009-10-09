/**
 * 
 */
package motivation.util.facades;

import java.util.List;
import java.util.concurrent.Callable;

import motivation.slice.Motive;
import motivation.slice.PlanProxy;
import motivation.util.castextensions.WMEntryQueue;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 *
 */
public class ExecutorFacade implements Callable<PlanProxy> { 
	
	/**
	 * @param component
	 */
	public ExecutorFacade(ManagedComponent component) {
		super();
		this.component = component;
	}

	List<Motive> motives;
	private ManagedComponent component;
	private WorkingMemoryAddress planAddr;

	public void setPlan(WorkingMemoryAddress planID) {
		this.planAddr=planID;
	}
	
	@Override
	public PlanProxy call() throws Exception {
		PlanProxy pp = new PlanProxy();
		pp.planAddress = planAddr;
		executePlan(pp);
		return pp;
	}
	

	private void executePlan(PlanProxy pp) throws InterruptedException {
		String id = component.newDataID();
		WMEntryQueue planProxyQueue = new WMEntryQueue(component);
		component.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.DELETE), planProxyQueue);
		// submit the planProxy
		try {
			component.addToWorkingMemory(id, pp);
			// wait for the PlanProxy to be deleted
			// TODO: we should move to status information in here and not only
			// listen for deletion
			planProxyQueue.take();
			component.log("plan proxy deletion seen");
			component.removeChangeFilter(planProxyQueue);
		} catch (CASTException e) {
			component.println("CASTException");
			e.printStackTrace();
		} catch (InterruptedException e) {
			component.println("executor should be interrupted");
			try {
				component.deleteFromWorkingMemory(id);
				throw (e);
			} catch (DoesNotExistOnWMException e1) {
				e1.printStackTrace();
			} catch (PermissionException e1) {
				e1.printStackTrace();
			}
		}
	}

}
