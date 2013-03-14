package motivation.components.managers;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.FutureTask;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import autogen.Planner.Completion;
import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import eu.cogx.planner.facade.PlannerFacade;

public class PlannerDispatcher extends CASTHelper implements Runnable {

	private static final int MINIMUM_PLANNER_TIMEOUT = 30;

	protected PlannerDispatcher(ManagedComponent c) {
		super(c);
		planner = PlannerFacade.get(c);
		jobsQueue = new LinkedBlockingQueue<PlannerDispatcher.PlanningJob>();
	}

	public interface PlanningJobCB {
		void plannedFor(PlanningJob job);
	}

	public class PlanningJob {
		Map<WorkingMemoryAddress, Motive> goalsToPlanFor = new HashMap<WorkingMemoryAddress, Motive>();
		Map<WorkingMemoryAddress, Motive> possibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
		Map<WorkingMemoryAddress, Motive> impossibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
		boolean scheduled = false;
		WMEntryQueueElement<PlanningTask> task;
		PlanningJobCB cb;
	}

	final LinkedBlockingQueue<PlanningJob> jobsQueue;
	final PlannerFacade planner;
	private boolean isPlanning = false;

	@Override
	public void run() {
		// TODO Auto-generated method stub
		while (component.isRunning()) {
			try {
				if (jobsQueue.isEmpty()) {
					synchronized (this) {
						isPlanning = false;
						notifyAll();
					}
				}
				log("jobs waiting in planner queue: " + jobsQueue.size());
				PlanningJob job = jobsQueue.take();
				log("planning for next job with " + job.goalsToPlanFor.size()
						+ " goals");
				synchronized (this) {
					isPlanning = true;
					notifyAll();
				}
				synchronized (job) {
					WMEntryQueueElement<PlanningTask> result = doPlanning(
							job.goalsToPlanFor, job.possibleGoals,
							job.impossibleGoals);
					job.task = result;
					job.scheduled = false;
					log("planning done... notify waiting threads");
					job.notifyAll();
				}
				if (job.cb != null) {
					log("call callback for this job");
					job.cb.plannedFor(job);
				}
			} catch (InterruptedException e) {
				component.getLogger().warn("jobsQueue.take got interrupted");
			} catch (ExecutionException e) {
				component.getLogger().warn(
						"ExecutionException: " + e.getMessage());
			}
		}

	}

	public WMEntryQueueElement<PlanningTask> plan(
			Map<WorkingMemoryAddress, Motive> goalsToPlanFor,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals)
			throws ExecutionException, InterruptedException {
		log("asked to schedule a new planning job and wait for it");
		FutureTask<WMEntryQueueElement<PlanningTask>> f = plan(goalsToPlanFor,
				possibleGoals, impossibleGoals, null);
		return f.get();
	}

	public WMEntryQueueElement<PlanningTask> plan(Motive motive)
			throws ExecutionException, InterruptedException {
		log("asked to schedule a new planning job and wait for it");
		Map<WorkingMemoryAddress, Motive> goalsToPlanFor = new HashMap<WorkingMemoryAddress, Motive>();
		Map<WorkingMemoryAddress, Motive> possibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
		Map<WorkingMemoryAddress, Motive> impossibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
		goalsToPlanFor.put(motive.thisEntry, motive);
		FutureTask<WMEntryQueueElement<PlanningTask>> f = plan(goalsToPlanFor,
				possibleGoals, impossibleGoals, null);
		return f.get();
	}

	public FutureTask<WMEntryQueueElement<PlanningTask>> plan(Motive motive,
			PlanningJobCB callback) throws ExecutionException,
			InterruptedException {
		log("asked to schedule a new planning job and wait for it");
		Map<WorkingMemoryAddress, Motive> goalsToPlanFor = new HashMap<WorkingMemoryAddress, Motive>();
		Map<WorkingMemoryAddress, Motive> possibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
		Map<WorkingMemoryAddress, Motive> impossibleGoals = new HashMap<WorkingMemoryAddress, Motive>();
		goalsToPlanFor.put(motive.thisEntry, motive);
		FutureTask<WMEntryQueueElement<PlanningTask>> f = plan(goalsToPlanFor,
				possibleGoals, impossibleGoals, callback);
		return f;
	}

	public FutureTask<WMEntryQueueElement<PlanningTask>> plan(
			Map<WorkingMemoryAddress, Motive> goalsToPlanFor,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals,
			PlanningJobCB callback) {
		final PlanningJob job = new PlanningJob();
		job.goalsToPlanFor = goalsToPlanFor;
		job.possibleGoals = possibleGoals;
		job.impossibleGoals = impossibleGoals;
		job.cb = callback;
		submitJob(job);
		return new FutureTask<WMEntryQueueElement<PlanningTask>>(
				new Callable<WMEntryQueueElement<PlanningTask>>() {

					@Override
					public WMEntryQueueElement<PlanningTask> call()
							throws Exception {
						synchronized (job) {
							job.wait();
							return job.task;
						}
					}
				});
	}

	public void submitJob(PlanningJob job) {
		synchronized (job) {
			job.scheduled = true;
			jobsQueue.add(job);
			log("new planning job submitted, new queue size is "
					+ jobsQueue.size());
		}
	}

	public boolean isScheduled(PlanningJob job) {
		synchronized (job) {
			return job.scheduled;
		}
	}

	public synchronized boolean isPlanning() {
		return isPlanning;
	}

	public synchronized void waitForIdle() throws InterruptedException {
		while (!isPlanning()) {
			log("waiting to notification in waitForIdle()");
			wait();
		}
	}

	protected WMEntryQueueElement<PlanningTask> doPlanning(
			Map<WorkingMemoryAddress, Motive> goalsToPlanFor,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals)
			throws InterruptedException, ExecutionException {
		int timeout = MINIMUM_PLANNER_TIMEOUT;
		for (Motive g : goalsToPlanFor.values()) {
			timeout += g.maxPlanningTime;
		}
		return doPlanning(goalsToPlanFor, possibleGoals, impossibleGoals,
				timeout);
	}

	protected WMEntryQueueElement<PlanningTask> doPlanning(
			Map<WorkingMemoryAddress, Motive> goalsToPlanFor,
			Map<WorkingMemoryAddress, Motive> possibleGoals,
			Map<WorkingMemoryAddress, Motive> impossibleGoals, int timeoutMs)
			throws InterruptedException, ExecutionException {
		log("try to plan for " + goalsToPlanFor.size() + " goals; timeout="
				+ timeoutMs + "ms.");
		Map<String, WorkingMemoryAddress> goal2motiveMap = new HashMap<String, WorkingMemoryAddress>();

		List<Goal> goals = new LinkedList<Goal>();
		for (Motive m : goalsToPlanFor.values()) {
			if (!m.goal.goalString.isEmpty())
				// set all those that are of HIGHEST priority to HARD goals
				if (m.priority == MotivePriority.HIGH)
					m.goal.importance = -1;
			goals.add(m.goal);
		}

		for (Entry<WorkingMemoryAddress, Motive> m : goalsToPlanFor.entrySet()) {
			if (!m.getValue().goal.goalString.isEmpty()) {
				if (goal2motiveMap.containsKey(m.getValue().goal.goalString)) {
					getLogger()
							.warn("the goal "
									+ m.getValue().goal.goalString
									+ " is already associated with motive"
									+ goal2motiveMap.get(m.getValue().goal.goalString));
					continue;
				} else {
					goal2motiveMap
							.put(m.getValue().goal.goalString, m.getKey());
				}
			}
		}

		impossibleGoals.clear();
		possibleGoals.clear();
		impossibleGoals.putAll(goalsToPlanFor);
		try {
			WMEntryQueueElement<PlanningTask> result = null;

			if (timeoutMs > 0) {
				result = planner.plan(goals).get(timeoutMs, TimeUnit.SECONDS);
			} else {
				result = planner.plan(goals).get();
			}
			if (result == null) {
				getLogger().warn("failed to create a plan at all");
				return null;
			}
			if (result.getEntry().planningStatus != Completion.SUCCEEDED) {
				getLogger().warn(
						"planner returned with result "
								+ result.getEntry().planningStatus.name());
				return null;
			}

			// mark all motives POSSIBLE that have been planned for
			for (Goal o : result.getEntry().goals) {
				WorkingMemoryAddress correspMotiveWMA = goal2motiveMap
						.get(o.goalString);
				assert (correspMotiveWMA != null);
				log("looking for planning result for goal " + o.goalString
						+ " (#actions=" + result.getEntry().plan.length + "): "
						+ o.isInPlan);

				if (o.isInPlan) {
					log("  goal " + o.goalString + " is possible!");
					possibleGoals.put(correspMotiveWMA,
							impossibleGoals.remove(correspMotiveWMA));
				}
			}

			return result;
		} catch (TimeoutException e) {
			getLogger().warn("planner timeout", e);
			return null;
		}
	}

}
