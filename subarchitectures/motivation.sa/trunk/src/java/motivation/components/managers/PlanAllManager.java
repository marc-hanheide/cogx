/**
 * 
 */
package motivation.components.managers;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.FutureTask;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import motivation.slice.CategorizePlaceMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.slice.PlanProxy;
import motivation.util.GoalTranslator;
import motivation.util.WMEntryQueue;
import motivation.util.WMMotiveEventQueue;
import motivation.util.WMMotiveSet;
import motivation.util.WMEntryQueue.QueueElement;
import motivation.util.WMEntrySet.ChangeHandler;
import motivation.util.WMMotiveSet.MotiveStateTransition;
import Ice.ObjectImpl;
import autogen.Planner.Completion;
import autogen.Planner.PlanningTask;
import binder.autogen.core.Feature;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.StringValue;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author marc
 * 
 */
public class PlanAllManager extends ManagedComponent {

	WMMotiveSet motives;
	volatile private boolean interrupt;
	private WMMotiveEventQueue activeMotiveEventQueue;

	Executor backgroundExecutor;
	private Union agentUnion;
	
	int exectionTimeoutSecs = 10;

	/**
	 * @param specificType
	 */
	public PlanAllManager() {
		super();
		motives = WMMotiveSet.create(this);
		activeMotiveEventQueue = new WMMotiveEventQueue();
		backgroundExecutor = Executors.newCachedThreadPool();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		// TODO Auto-generated method stub
		super.start();
		log("start up");
		motives.start();
		motives.setStateChangeHandler(new MotiveStateTransition(null,
				MotiveStatus.ACTIVE), activeMotiveEventQueue);
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
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> arg0) {
		super.configure(arg0);
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
					if (activeMotiveEventQueue.poll(1, TimeUnit.SECONDS) != null)
						break;
				}
				log("received relevant change");
				interrupt = false;
				final List<Motive> activeMotives = new LinkedList<Motive>(
						motives.getSubsetByStatus(MotiveStatus.ACTIVE));

				if (activeMotives.size() > 0) { // if we have motives to
					log("we have some motives that should be planned for");
					FutureTask<QueueElement> generatedPlan = new FutureTask<QueueElement>(
							new Callable<QueueElement>() {
								public QueueElement call() {
									try {
										return generatePlan(activeMotives);
									} catch (InterruptedException e) {
										println("planning was interrupted");
									}
									return null;
								}
							});

					// generate the plan asynchronously
					backgroundExecutor.execute(generatedPlan);

					// wait for the future to be completed
					QueueElement pt = null;
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

					if (!generatedPlan.isDone())
						generatedPlan.cancel(true);

					if (pt != null) { // if we got a plan...
						log("a plan has been generated. it's time to execute it");
						final PlanProxy pp = new PlanProxy();
						pp.planAddress = pt.getEvent().address;
						FutureTask<Object> executionResult = new FutureTask<Object>(
								new Callable<Object>() {
									@Override
									public Object call() {
										try {
											executePlan(pp);
										} catch (InterruptedException e) {
											log("execution has been interrupted");
										} finally {
											deactivateMotives();
										}
										return null;
									}
								});
						backgroundExecutor.execute(executionResult);
						int loopCount=0;
						while (!interrupt) {
							try {
								executionResult.get(1, TimeUnit.SECONDS);
								// interrupt any execution after timeout
								break;
							} catch (TimeoutException e) {
								log("not finished execution yet... continue waiting");
								if (++loopCount>exectionTimeoutSecs) {
									log("timeout in execution");
									interrupt=true;
								}
							}

						}
						if (!executionResult.isDone())
							executionResult.cancel(true);
					}
					log("execution finished... wait 1 sec to let state changes propagate");
					sleepComponent(1000); // TODO: wait for motives to
					// update
				} else { // if we have no motives yet, we wait until something
					// happens
					waitForChanges();
				}
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (ExecutionException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	/**
	 * Create task with non-crashy default values.
	 * 
	 * @return
	 */
	private PlanningTask newPlanningTask() {
		return new PlanningTask(0, null, null, null, null, Completion.PENDING,
				0, Completion.PENDING, 0);
	}

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

	/**
	 * @throws InterruptedException
	 * 
	 */
	private synchronized QueueElement generatePlan(List<Motive> activeMotives)
			throws InterruptedException {
		WMEntryQueue planQueue = new WMEntryQueue(this);
		QueueElement pt = null;

		// if we don't have anything to do... just quit.
		if (activeMotives.size() < 0)
			return null;

		try {
			String id = newDataID();
			PlanningTask plan = newPlanningTask();

			// create a conjunction of motives
			String goalString = "(and ";
			for (Motive m : activeMotives) {
				if (m instanceof ExploreMotive) {
					log("---> add goal to explore node "
							+ ((ExploreMotive) m).placeID);
					goalString = goalString
							+ GoalTranslator
									.motive2PlannerGoal((ExploreMotive) m);
				} else if (m instanceof HomingMotive) {
					log("---> add goal to go home "
							+ ((HomingMotive) m).homePlaceID);
					goalString = goalString
							+ GoalTranslator
									.motive2PlannerGoal((HomingMotive) m);
				} else if (m instanceof CategorizePlaceMotive) {

					log("---> add goal to categorize place "
							+ ((CategorizePlaceMotive) m).placeID);
					goalString = goalString
							+ GoalTranslator.motive2PlannerGoal(
									(CategorizePlaceMotive) m, getAgentUnion());
				}
			}

			goalString = goalString + ")";
			log("generated goal string: " + goalString);
			plan.goal = goalString;
			// plan.goal = "(forall (?p - place) (= (explored ?p) true))";

			addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.OVERWRITE), planQueue);
			addChangeFilter(ChangeFilterFactory.createIDFilter(id,
					WorkingMemoryOperation.DELETE), planQueue);
			log("submitting plan to WM");
			addToWorkingMemory(id, plan);
			log("has been submitted");

			// wait for the plan to be generated
			boolean continueWaiting = true;
			while (continueWaiting) {
				log("waiting for planner to answer");
				pt = planQueue.take();
				PlanningTask taskEntry = (PlanningTask) pt.getEntry();
				if (pt.getEntry() == null) {
					pt = null;
					println("the Planning task has been removed before we actually received a valid plan...");
					break;
				}
				switch (taskEntry.planningStatus) {
				case SUCCEEDED:
					log("we have a plan right now: "
							+ ((PlanningTask) pt.getEntry()).goal);
					// stop waiting for further changes
					continueWaiting = false;
					break;
				case ABORTED:
				case FAILED:
					log("could not generate a plan... we have to abort for this time and remove the listener");
					removeChangeFilter(planQueue);
					planQueue = null;
					deleteFromWorkingMemory(pt.getEvent().address);
					pt = null;
					// stop waiting for further changes
					continueWaiting = false;
					break;
				default:
					log("still planning... continue waiting with status "
							+ taskEntry.planningStatus.name());
				}
			}

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		} finally {
			try {
				if (planQueue != null) {
					log("remove listener");
					removeChangeFilter(planQueue);
				}
			} catch (SubarchitectureComponentException e) {
				println("SubarchitectureComponentException");
				e.printStackTrace();
			}
		}
		return pt;
	}

	
	private void executePlan(PlanProxy pp) throws InterruptedException {
		String id = newDataID();
		WMEntryQueue planProxyQueue = new WMEntryQueue(this);
		addChangeFilter(ChangeFilterFactory.createIDFilter(id,
				WorkingMemoryOperation.DELETE), planProxyQueue);
		// submit the planProxy
		try {
			addToWorkingMemory(id, pp);
			// wait for the PlanProxy to be deleted
			// TODO: we should move to status information in here and not only
			// listen for deletion
			planProxyQueue.take();
			removeChangeFilter(planProxyQueue);
		} catch (CASTException e) {
			println("CASTException");
			e.printStackTrace();
		} catch (InterruptedException e) {
			println("executor should be interrupted");
			try {
				deleteFromWorkingMemory(id);
				throw (e);
			} catch (DoesNotExistOnWMException e1) {
				e1.printStackTrace();
			} catch (PermissionException e1) {
				e1.printStackTrace();
			}
		}
	}

	private Union getAgentUnion() throws UnknownSubarchitectureException {
		List<UnionConfiguration> l = new LinkedList<UnionConfiguration>();
		getMemoryEntries(UnionConfiguration.class, l,"binder");
		// TODO: we ugly search for the agent union
		Union[] unions = l.get(0).includedUnions;

		if (agentUnion != null)
			return agentUnion;
		else {
			for (Union u : unions) {
				for (Feature f : u.features) {
					if (f.featlabel.equals("category")) {
						if (f.alternativeValues.length > 0)
							if (((StringValue) f.alternativeValues[0]).val
									.equals("robot")) {
								agentUnion = u;
								return agentUnion;
							}
					}
				}
			}
		}
		return null;

	}

}
