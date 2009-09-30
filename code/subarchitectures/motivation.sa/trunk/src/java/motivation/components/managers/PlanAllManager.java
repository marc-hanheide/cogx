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
import motivation.slice.PlanProxy;
import motivation.util.PlaceUnionEventRelation;
import motivation.util.WMMotiveEventQueue;
import motivation.util.WMMotiveSet;
import motivation.util.WMMotiveSet.MotiveStateTransition;
import motivation.util.castextensions.WMEntryQueue.WMEntryQueueElement;
import motivation.util.castextensions.WMEntrySet.ChangeHandler;
import motivation.util.facades.BinderFacade;
import motivation.util.facades.ExecutorFacade;
import motivation.util.facades.PlannerFacade;
import Ice.ObjectImpl;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;

/**
 * @author marc
 * 
 */
public class PlanAllManager extends ManagedComponent {

	WMMotiveSet motives;
	volatile private boolean interrupt;
	private WMMotiveEventQueue activeMotiveEventQueue;

	PlaceUnionEventRelation placeUnionEventRelation;
	PlannerFacade plannerFacade;
	ExecutorFacade executorFacade;
	BinderFacade binderFacade;

	Executor backgroundExecutor;

	int failsafeTimeoutSecs = 100;

	/**
	 * @param specificType
	 */
	public PlanAllManager() {
		super();
		motives = WMMotiveSet.create(this);
		binderFacade = new BinderFacade(this);
		plannerFacade = new PlannerFacade(this, binderFacade);
		executorFacade = new ExecutorFacade(this);
		activeMotiveEventQueue = new WMMotiveEventQueue();
		backgroundExecutor = Executors.newCachedThreadPool();
		placeUnionEventRelation = new PlaceUnionEventRelation(this);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		log("start up");
		binderFacade.start();
		motives.start();
		// register listener for state transition to ACTIVE (trigger processing
		// whenever some motive becomes ACTIVE)
		motives.setStateChangeHandler(new MotiveStateTransition(null,
				MotiveStatus.ACTIVE), activeMotiveEventQueue);
		// causing an interrupt is some motive is de-activated by someone
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.ACTIVE, null), new ChangeHandler() {

			@Override
			public void motiveChanged(
					Map<WorkingMemoryAddress, ObjectImpl> map,
					WorkingMemoryChange wmc, ObjectImpl newMotive,
					ObjectImpl oldMotive) {
				interrupt = true;

			}
		});
		// start the causal event listening on places
		placeUnionEventRelation.start();
	}

	@Override
	protected void configure(Map<String, String> arg0) {
		log("configure manager");
		super.configure(arg0);
		String valStr;
		if ((valStr = arg0.get("--failsafetimeout")) != null)
			failsafeTimeoutSecs = Integer.parseInt(valStr);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#runComponent()
	 */
	@Override
	protected void runComponent() {
		log("running");
		try {
			while (isRunning()) {
				log("runComponent loop");
				while (isRunning()) {
					// wait that something happens
					if (activeMotiveEventQueue.poll(1, TimeUnit.SECONDS) != null) {
						// make sure all changes have been propagated to the unions!
						log("wait to finialize propagation to unions");
						placeUnionEventRelation.waitForPropagation();
						// after this we can be quite sure that we actually have all required information on the binder
						break;
					}
				}
				log("received relevant change");
				interrupt = false;
				List<Motive> activeMotives = new LinkedList<Motive>(motives
						.getSubsetByStatus(MotiveStatus.ACTIVE));

				if (activeMotives.size() > 0) { // if we have motives to
					log("we have some motives that should be planned for");
					// activeMotives = resolveMotives(activeMotives);
					plannerFacade.setGoalMotives(activeMotives);
					FutureTask<WMEntryQueueElement> generatedPlan = new FutureTask<WMEntryQueueElement>(
							plannerFacade);
					// generate the plan asynchronously
					backgroundExecutor.execute(generatedPlan);
					// wait for the future to be completed
					WMEntryQueueElement pt = null;
					while (!interrupt) {
						try {
							pt = generatedPlan.get(1, TimeUnit.SECONDS);
							break;
						} catch (TimeoutException e) {
							log("no plan yet... continue waiting");
						}
						// TODO we might want to interrupt planning here for a
						// good reason
						// generatedPlan.cancel(true);
					}

					// probably cancel the planning task
					if (!generatedPlan.isDone())
						generatedPlan.cancel(true);

					if (pt != null) { // if we got a plan...
						log("a plan has been generated. it's time to execute it");
						executorFacade.setPlan(pt.getEvent().address);
						FutureTask<PlanProxy> executionResult = new FutureTask<PlanProxy>(
								executorFacade);
						backgroundExecutor.execute(executionResult);
						int loopCount = 0;
						while (!interrupt) {
							try {
								executionResult.get(1, TimeUnit.SECONDS);
								// interrupt any execution after timeout
								break;
							} catch (TimeoutException e) {
								log("not finished execution yet... continue waiting");
								if (++loopCount > failsafeTimeoutSecs) {
									log("timeout in execution");
									interrupt = true;
								}
							}

						}
						if (!executionResult.isDone())
							executionResult.cancel(true);
						log("execution finished... wait 1 sec to let state changes propagate");
					} else {
						deactivateMotives();
					}
					sleepComponent(1000); // TODO: wait for motives to
					// update
				}
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (ExecutionException e) {
			e.printStackTrace();
		}

	}

	// /**
	// * Create task with non-crashy default values.
	// *
	// * @return
	// */
	// private PlanningTask newPlanningTask() {
	// return new PlanningTask(0, null, null, null, null, Completion.PENDING,
	// 0, Completion.PENDING, 0);
	// }

	void deactivateMotives() {
		try {
			for (Motive m : motives.getSubsetByStatus(MotiveStatus.ACTIVE)) {
				m.status = MotiveStatus.SURFACED;
				overwriteWorkingMemory(m.thisEntry, m);
			}
		} catch (DoesNotExistOnWMException e) {
			log("deactive a motive that doesn't exist... no worries");
		} catch (CASTException e) {
			e.printStackTrace();
		}

	}

}
