/**
 * @author Team RED
 */

package css11.red.planning;

import cast.architecture.ManagedComponent;

import org.apache.log4j.Logger;

import mathlib.Functions;

import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTData;
import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Rect2;
import cogx.Math.Sphere3;
import cogx.Math.Vector2;
import cogx.Math.Vector3;

import java.util.List;
import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;

import css11.red.planning.slice.Action;
import css11.red.planning.slice.Completion;
import css11.red.planning.slice.Plan;

public class PlanExecutor extends ManagedComponent {

	private WorkingMemoryAddress plan_wma = null;
	private Vector<Action> cur_plan = new Vector<Action>();
	private int plan_idx = 0;
	private Completion cur_comp = Completion.NOTFINISHED;

	private WorkingMemoryChangeReceiver recv = null;
	private WorkingMemoryPointer current_plan = null;

	private int ncsleep = 500;

	@Override
	protected void configure(Map<String, String> config) {
		if (config.containsKey("--ncsleep")) {
			String s = config.get("--ncsleep");
			try {
				int i = Integer.parseInt(s);
				ncsleep = i;
			}
			catch (NumberFormatException ex) {
				log("couldn't parse \"" + s + "\"");
			}
		}
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				Plan.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleNewPlan(_wmc);
					}
				});
	}

	@Override
	protected void runComponent() {
		//verbalize("Plan executor ready.");

		while (this.isRunning()) {
			try {
				Thread.sleep(200);
			}
			catch (InterruptedException ex) {
				log(ex);
			}

			boolean flush = false;

			// we're currently monitoring an action
			if (getCurrentAction() != null) {

				if (getCurrentAction().comp == Completion.SUCCESS) {
					// the action succeeded
					log("command succeeded, will proceed with the next one");
					stopMonitoringCurrentAction();
				}
				else if (getCurrentAction().comp == Completion.FAILURE) {
					// the action failed
					log("command failed, failing the entire plan");
					stopMonitoringCurrentAction();
					cur_comp = Completion.FAILURE;
				}
				else {
					// the action is still executing
				}
			}

			// we're not monitoring any action now, let's look at the plan
			if (getCurrentAction() == null) {
				if (plan_wma != null) {
					if (cur_comp == Completion.NOTFINISHED) {
						// apparently we still have a plan
						if (nextAction()) {
							log("getting the next action (" + cur_plan.size() + " items left after this one)");
							startMonitoringCurrentAction(getCurrentAction());
						}
						else {
							// plan empty
							log("plan successfully executed");
							cur_comp = Completion.SUCCESS;
						}
					}

					if (cur_comp != Completion.NOTFINISHED) {
						finalizeCurrentPlan(cur_comp);
						resetCurrentPlan();
					}
				}
			}
		}
	}

	synchronized void startMonitoringCurrentAction(Action a) {
		log("adding new action; will sleep for " + ncsleep + " ms");
		setCurrentAction(a);
		String id = newDataID();

		try {
			Thread.sleep(ncsleep);
			log("now adding the action");
			addToWorkingMemory(id, a);
			if (recv == null) {

				recv = new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(WorkingMemoryChange _wmc) {
							handleActionOverwrite(_wmc);
						}
				};

				addChangeFilter(ChangeFilterFactory.createAddressFilter(
						id, getSubarchitectureID(), WorkingMemoryOperation.OVERWRITE),
						recv);
			}
			else {
				log("WARNING WARNING NUCLEAR ATTACK, recv != NULL (I'm not registering it) !!!");
			}
		}
		catch (InterruptedException ex) {
			log(ex);
		}
		catch (AlreadyExistsOnWMException ex) {
			log(ex);
		}
	}

	synchronized void overwriteCurrentAction(Action a) {
		setCurrentAction(a);
	}

	synchronized void stopMonitoringCurrentAction() {
		if (getCurrentAction() != null) {
			log("stopping the monitoring of the current action");

			if (recv != null) {
				try {
					removeChangeFilter(recv);
				}
				catch (SubarchitectureComponentException ex) {
					log(ex);
				}
			}

			recv = null;
		}
		else {
			log("currently not monitoring any action");
		}
	}

	public void handleActionOverwrite(WorkingMemoryChange wmc) {
		log("woohoo, detected a change on address [" + wmc.address.id + "]");
		try {
			CASTData data = getWorkingMemoryEntry(wmc.address.id);
			if (data.getData() instanceof Action) {
				Action a = (Action) data.getData();
				overwriteCurrentAction(a);
			}
			else {
				log("oops, got something that isn't an Action (ignoring this)!");
			}
		}
		catch (SubarchitectureComponentException ex) {
			log(ex);
		}
	}

	synchronized void resetCurrentPlan() {
		cur_plan.clear();
		cur_comp = Completion.NOTFINISHED;
		plan_wma = null;
	}

	public void handleNewPlan(WorkingMemoryChange wmc) {
		log("got a new plan");

		finalizeCurrentPlan(Completion.FAILURE);

		try {
			Plan p = getMemoryEntry(wmc.address, Plan.class);
			assert (p != null);

			for (int i = 0; i < p.actions.length; i++) {
				cur_plan.add(p.actions[i]);
			}
			plan_wma = wmc.address;
		}
		catch (SubarchitectureComponentException ex) {
			log(ex);
		}
	}

	public void finalizeCurrentPlan(Completion comp) {
		if (plan_wma != null) {
			try {
				Plan p = new Plan(comp, cur_plan.toArray(new Action[0]));
				overwriteWorkingMemory(plan_wma, p);
			}
			catch (SubarchitectureComponentException ex) {
				log(ex);
			}
		}
		resetCurrentPlan();
	}

	synchronized Action getCurrentAction() {
		if (plan_idx >= 0 && plan_idx < cur_plan.size()) {
			return cur_plan.get(plan_idx);
		}
		else {
			return null;
		}
	}

	synchronized void setCurrentAction(Action new_a) {
		Action a = getCurrentAction();
		if (a != null) {
			a = new_a;
		}
	}

	synchronized boolean nextAction() {
		if (plan_idx >= 0 && plan_idx < cur_plan.size() - 1) {
			plan_idx++;
			return true;
		}
		else {
			return false;
		}
	}

}
