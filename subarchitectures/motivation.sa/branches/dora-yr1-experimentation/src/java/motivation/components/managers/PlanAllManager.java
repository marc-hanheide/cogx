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
import motivation.util.RoomUnionEventRelation;
import motivation.util.WMMotiveEventQueue;
import motivation.util.WMMotiveSet;
import motivation.util.WMMotiveSet.MotiveStateTransition;
import motivation.util.castextensions.WMLock;
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
import cast.cdl.WorkingMemoryPermissions;
import motivation.util.castextensions.WMLogger;

/**
 * @author marc
 * 
 */
public class PlanAllManager extends ManagedComponent {

	WMMotiveSet motives;
	WMLock wmLock;
	volatile private boolean interrupt;
	private WMMotiveEventQueue activeMotiveEventQueue;

	PlaceUnionEventRelation placeUnionEventRelation;
	RoomUnionEventRelation roomUnionEventRelation;
	PlannerFacade plannerFacade;
	ExecutorFacade executorFacade;
	BinderFacade binderFacade;

	Executor backgroundExecutor;

	/** this is the the very last resort time out... if a plan has been executed in this amount of time we break
	 * 
	 */
	int failsafeExectutionTimeoutSecs = 600;

	/** this is the the very last resort time out... if a plan has not been delivered in time 
	 * 
	 */
	int failsafePlanningTimeoutSecs = 10;

	/**
	 * @param specificType
	 * @throws CASTException 
	 */
	public PlanAllManager() throws CASTException {
		super();
		wmLock = new WMLock(this, "SchedulerManagerSync");
		motives = WMMotiveSet.create(this);
		binderFacade = new BinderFacade(this);
		plannerFacade = new PlannerFacade(this, binderFacade);
		executorFacade = new ExecutorFacade(this);
		activeMotiveEventQueue = new WMMotiveEventQueue();
		backgroundExecutor = Executors.newCachedThreadPool();
		placeUnionEventRelation = new PlaceUnionEventRelation(this);
		roomUnionEventRelation = new RoomUnionEventRelation(this);
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
		// register listener for state transition to ACTIVE (trigger processing
		// whenever some motive becomes ACTIVE)
		motives.setStateChangeHandler(new MotiveStateTransition(null,
				MotiveStatus.ACTIVE), activeMotiveEventQueue);
		// causing an interrupt if some motive is de-activated by someone
		motives.setStateChangeHandler(new MotiveStateTransition(
				MotiveStatus.ACTIVE, null), new ChangeHandler() {

			@Override
			public void entryChanged(
					Map<WorkingMemoryAddress, ObjectImpl> map,
					WorkingMemoryChange wmc, ObjectImpl newMotive,
					ObjectImpl oldMotive) {
				interrupt = true;

			}
		});

		binderFacade.start();
		motives.start();
		// start the causal event listening on places
		placeUnionEventRelation.start();
		roomUnionEventRelation.start();
	}

	@Override
	protected void configure(Map<String, String> arg0) {
		log("configure manager");
		String valStr;
		if ((valStr = arg0.get("--failsafetimeout")) != null)
			failsafeExectutionTimeoutSecs = Integer.parseInt(valStr);
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
			wmLock.initialize();
			while (isRunning()) {
				log("checking for active motives to manage them");
				// flush queue
				while (!activeMotiveEventQueue.isEmpty())
					activeMotiveEventQueue.poll();

				interrupt = false;
				List<Motive> activeMotives = null;
				try {
					wmLock.lock();
					activeMotives = new LinkedList<Motive>(motives
							.getMapByStatus(MotiveStatus.ACTIVE).values());
				} finally {
					wmLock.unlock();
				}

				if (activeMotives.size() > 0) { // if we have motives to
					log("we have some motives that should be planned for");

					// make sure all changes have been propagated to the
					// unions!
					log("wait to finalize propagation of places to unions");
					placeUnionEventRelation.waitForPropagation(1000);
					log("wait to finalize propagation of rooms to unions");
					roomUnionEventRelation.waitForPropagation(1000);
					log("got all changes");
					// after this we can be quite sure that we actually have
					// all required information on the binder, available to the
					// planner

					plannerFacade.setGoalMotives(activeMotives);
					FutureTask<WMEntryQueueElement> generatedPlan = new FutureTask<WMEntryQueueElement>(
							plannerFacade);
					// generate the plan asynchronously
					backgroundExecutor.execute(generatedPlan);
					// in the meantime compute the maximum time to wait for the plan
					int maxPlanningTime=0;
					int maxExecutionTime=0;
					for (Motive m : activeMotives) {
						maxPlanningTime+=m.maxPlanningTime;
						maxExecutionTime+=m.maxExecutionTime;
					}
					log("max planning and execution time for problem computed:\n" 
							+ "  maxPlanningTime == " + maxPlanningTime +"\n"
							+ "  maxExecutionTime == " + maxExecutionTime);	
					
					// wait for the future to be completed
					WMEntryQueueElement pt = null;
					int loopCount = 0;
					while (!interrupt) {
						try {
							pt = generatedPlan.get(1, TimeUnit.SECONDS);
							break;
						} catch (TimeoutException e) {
							log("no plan yet... continue waiting");
							if (++loopCount > maxPlanningTime) {
								log("timeout in planning");
                                                                getLogger().info("<MOTIVETIMEOUT during=\"planning\"/ cast_time=\""+WMLogger.CASTTimeToString(getCASTTime())+"\">");
								interrupt = true;
							}
						}
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

						loopCount = 0;
						while (!interrupt) {
							try {
								executionResult.get(1, TimeUnit.SECONDS);
								// interrupt any execution after timeout
								Thread.sleep(1000);
								break;
							} catch (TimeoutException e) {
								log("not finished execution yet... continue waiting");
								if (++loopCount > maxExecutionTime) {
									log("timeout in execution");
                                                                        getLogger().info("<MOTIVETIMEOUT during=\"execution\"/ cast_time=\""+WMLogger.CASTTimeToString(getCASTTime())+"\">");
									interrupt = true;
								}
							}

						}

						if (!executionResult.isDone()) {
							log("cancelling execution");
							executionResult.cancel(true);
						}
					} else {
						log("no plan available, deactivating motives");
					}

					// deactivate motives
					deactivateMotives();
					log("wait 1 sec to let state changes propagate");
					sleepComponent(1000); // TODO: wait for motives to
					// update
				} else {
					log("there are no active motives right now... sleeping until some turn up");
					// wait that something happens
					while (isRunning()) {
						if (activeMotiveEventQueue.poll(1, TimeUnit.SECONDS) != null) {
							activeMotiveEventQueue.clear();
							log("received relevant event");
							break;
						}
					}
				}
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (ExecutionException e) {
			e.printStackTrace();
		} catch (CASTException e) {
			// TODO Auto-generated catch block
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
		for (Motive m : motives.getMapByStatus(MotiveStatus.ACTIVE).values()) {
			m.status = MotiveStatus.SURFACED;
			try {
				lockEntry(m.thisEntry, WorkingMemoryPermissions.LOCKEDO);
				getMemoryEntry(m.thisEntry, Motive.class);
				overwriteWorkingMemory(m.thisEntry, m);

			} catch (DoesNotExistOnWMException e) {
				log("deactive a motive that doesn't exist... no worries");
			} catch (CASTException e) {
				e.printStackTrace();
			} finally {
				try {
					unlockEntry(m.thisEntry);
				} catch (CASTException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}

	}

}
