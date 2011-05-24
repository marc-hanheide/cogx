/**
 * 
 */
package facades;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.FutureTask;

import motivation.slice.PlanProxy;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMEntryQueue;

/**
 * @author marc
 * 
 */
public class ExecutorFacade extends CASTHelper {

	public class ExecutionCallable implements Callable<PlanProxy> {

		private PlanProxy planProxy;

		public ExecutionCallable(WorkingMemoryAddress planID) {
			planProxy = new PlanProxy(planID);
		}

		@Override
		public PlanProxy call() throws Exception {
			String id = component.newDataID();
			WMEntryQueue<PlanProxy> planProxyQueue = new WMEntryQueue<PlanProxy>(
					component, PlanProxy.class);
			component.addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.DELETE), planProxyQueue);
			// submit the planProxy
			component.addToWorkingMemory(id, planProxy);
			// wait for the PlanProxy to be deleted
			// TODO: we should move to status information in here and not
			// only
			// listen for deletion
			planProxyQueue.take();
			getLogger().debug("plan proxy deletion seen");
			component.removeChangeFilter(planProxyQueue);
			for (Callable<?> c : listeners) {
				c.call();
			}
			return planProxy;

		}

	}

	public class FutureExecutionTask extends FutureTask<PlanProxy> {

		public FutureExecutionTask(WorkingMemoryAddress planID) {
			super(new ExecutionCallable(planID));
		}

	}

	final Set<Callable<?>> listeners = new HashSet<Callable<?>>();

	private final static ExecutorService executorService = Executors
			.newCachedThreadPool();

	/**
	 * @param component
	 */
	public ExecutorFacade(ManagedComponent component) {
		super(component);
	}

	public Future<PlanProxy> execute(WorkingMemoryAddress planID)
			throws CASTException {
		PlanningTask pt = component.getMemoryEntry(planID, PlanningTask.class);
		if (!pt.executePlan) {
			pt.executePlan = true;
			getLogger().info("setting planning task active");
			component.overwriteWorkingMemory(planID, pt);
		}
		try {
			getLogger().info("waiting a second as part of a big HACK!");
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			component.logException(e);
		}
		getLogger().info("starting execution for planID "
				+ CASTUtils.toString(planID));
		FutureExecutionTask futureExecution = new FutureExecutionTask(planID);
		executorService.execute(futureExecution);
		return futureExecution;
	}

	/**
	 * register callables to be called on completion of the executed plan
	 * 
	 * @param c
	 */
	public void registerCallable(Callable<?> c) {
		listeners.add(c);
	}

}
