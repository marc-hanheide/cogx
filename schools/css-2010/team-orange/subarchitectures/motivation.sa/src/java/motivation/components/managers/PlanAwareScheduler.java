/**
 * 
 */
package motivation.components.managers;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.FutureTask;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveEventQueue;
import motivation.util.WMMotiveSet;
import motivation.util.WMMotiveEventQueue.MotiveEvent;
import motivation.util.WMMotiveSet.MotiveStateTransition;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import castutils.facades.BinderFacade;
import castutils.facades.PlannerFacade;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.architecture.ManagedComponent;
import cast.core.CASTUtils;

/**
 * @author marc
 * 
 */
public class PlanAwareScheduler extends ManagedComponent {
	WMMotiveSet motives;

	WMMotiveEventQueue relevantEventQueue;

	BinderFacade binderFacade;

	PlannerFacade plannerFacade;
	Executor backgroundExecutor;
	volatile private boolean interrupt;

	/**
	 * @param specificType
	 * @throws CASTException
	 */
	public PlanAwareScheduler() throws CASTException {
		super();
		// defaults
		motives = WMMotiveSet.create(this);
		relevantEventQueue = new WMMotiveEventQueue();
		binderFacade = new BinderFacade(this);
		plannerFacade = new PlannerFacade(this, binderFacade);
		backgroundExecutor = Executors.newCachedThreadPool();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> arg0) {
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		motives.start();
		binderFacade.start();
		motives.setStateChangeHandler(new MotiveStateTransition(null,
				MotiveStatus.SURFACED), relevantEventQueue);
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.SURFACED, MotiveStatus.SURFACED),
				relevantEventQueue);
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.POSSIBLE, MotiveStatus.POSSIBLE),
				relevantEventQueue);
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.IMPOSSIBLE, MotiveStatus.IMPOSSIBLE),
				relevantEventQueue);
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.COMPLETED, MotiveStatus.COMPLETED),
				relevantEventQueue);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		super.runComponent();
		log("runComponent loop");
		while (isRunning()) {
			try {
				MotiveEvent event = relevantEventQueue.take();
				// avoid self calls
				if (event.wmc.src.equals(getComponentID()))
					continue;

				Motive motive = (Motive) event.newMotive;
				List<Motive> motives = new LinkedList<Motive>();
				motives.add(motive);
				plannerFacade.setGoalMotives(motives);
				FutureTask<WMEntryQueueElement> generatedPlan = new FutureTask<WMEntryQueueElement>(
						plannerFacade);
				// generate the plan asynchronously
				backgroundExecutor.execute(generatedPlan);
				// wait for the future to be completed
				WMEntryQueueElement pt = null;
				interrupt = false;
				log("trying to generate a plan");
				while (!interrupt) {
					try {
						pt = generatedPlan.get(1, TimeUnit.SECONDS);
						break;
					} catch (TimeoutException e) {
						log("no plan yet... continue waiting");
					} catch (ExecutionException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
				log("plan generation finished");

				if (pt != null) { // if we got a plan...
					PlanningTask taskEntry = (PlanningTask) pt.getEntry();
					motive.plannedCosts=taskEntry.plan.length;
					if (taskEntry.plan.length == 0) { // the plan is empty
						log("there is nothing to be done for motive "
								+ CASTUtils.toString(motive.thisEntry)
								+ ". It's COMPLETED ===");
						motive.status = MotiveStatus.COMPLETED;
					} else {
						log("motive " + CASTUtils.toString(motive.thisEntry)
								+ "is POSSIBLE +++");
						motive.status = MotiveStatus.POSSIBLE;
					}
				} else { // no plan available
					log("motive " + CASTUtils.toString(motive.thisEntry)
							+ "is IMPOSSIBLE ---");
					motive.status = MotiveStatus.IMPOSSIBLE;
				}
				overwriteWorkingMemory(motive.thisEntry, motive);

			} catch (InterruptedException e1) {
			} catch (CASTException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
	}
}
