/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.planner.facade;

import java.util.Map;
import java.util.List;
import java.util.LinkedList;
import java.util.StringTokenizer;

import javax.swing.table.TableModel;

import motivation.slice.PlanProxy;

import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMView;
import castutils.castextensions.WMEntryQueue.WMEntryQueueElement;
import castutils.viewer.WMJTableModel;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.planner.facade.ManualPlanningTaskFrame.SubmitListener;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class ManualPlanningTaskComponent extends ManagedComponent implements
		SubmitListener {
	public static final String OVERSUBSCRIPTION_MODE = "--oversubscription-mode";

	private static final String CONFIG_GOALS = "--goals";

	ManualPlanningTaskFrame frame;

	protected ManualPlanningTaskFrame getFrame() {
		return frame;
	}

	WMView<GroundedBelief> view;
	PlannerFacade planner;
	List<String> goals = null;

	private boolean m_oversubscription;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		goals = new LinkedList<String>();
		if (config.containsKey(CONFIG_GOALS)) {
			StringTokenizer st = new StringTokenizer(config.get(CONFIG_GOALS),
					";");
			while (st.hasMoreTokens()) {
				goals.add(st.nextToken());
			}
		}
		if (config.containsKey(OVERSUBSCRIPTION_MODE))
			m_oversubscription = true;
		else
			m_oversubscription = false;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		planner = new PlannerFacade(this);
		view = WMView.create(this, GroundedBelief.class);
		String[] columns = new String[] { "ID", "Type" };
		WMJTableModel<GroundedBelief> model = new WMJTableModel<GroundedBelief>(
				view, columns) {

			/**
			 * 
			 */
			private static final long serialVersionUID = 1L;

			@Override
			protected Object getValueAt(GroundedBelief object, int columnIndex) {
				switch (columnIndex) {
				case 0:
					return object.id;
				case 1:
					return object.type;
				default:
					return null;
				}
			}
		};
		frame = new ManualPlanningTaskFrame(model, this, goals);
		try {
			view.registerHandler(model);
			view.start();
		} catch (UnknownSubarchitectureException e) {
			logException(e);
		}
		frame.pack();
		frame.setVisible(true);
	}

	@Override
	public String submit(TableModel goalsTable, boolean shouldExecutePlan) {
		log("submit:" + goalsTable);
		if (m_oversubscription)
			return submitAllGoals(goalsTable, shouldExecutePlan);
		else
			return submitOneGoal(goalsTable, shouldExecutePlan);

	}

	private String submitOneGoal(TableModel goalsTable,
			boolean shouldExecutePlan) {
		LinkedList<Goal> goals = new LinkedList<Goal>();
		Goal g = new Goal(-1, -1, this.frame.getjGoalTextEditField().getText(),
				false);
		goals.add(g);

		try {
			WMEntryQueueElement<PlanningTask> res = planner.plan(goals,
					shouldExecutePlan).get();
			if (res == null)
				return "PLANNING FAILED";
			else {
				if (shouldExecutePlan) {
					addToWorkingMemory(newDataID(), new PlanProxy(res
							.getEvent().address));
				}
				String resultString = res.getEntry().planningStatus.name()
						+ " costs=" + res.getEntry().costs + ", goals:";
				for (Goal gr : res.getEntry().goals) {
					resultString += " [" + gr.goalString + ", " + gr.isInPlan
							+ "]";
				}
				return resultString;
			}
		} catch (Exception e) {
			logException(e);
			return e.getMessage();
		}
	}

	private String submitAllGoals(TableModel goalsTable,
			boolean shouldExecutePlan) {
		LinkedList<Goal> goals = new LinkedList<Goal>();
		for (int i = 0; i < goalsTable.getRowCount(); i++) {
			Object goalObj = goalsTable.getValueAt(i, 0);
			Object impObj = goalsTable.getValueAt(i, 1);
			Object deadlineObj = goalsTable.getValueAt(i, 2);
			if (goalObj != null && goalObj instanceof String
					&& ((String) goalObj).length() > 0) {
				int importance = -1;
				if (impObj != null && impObj instanceof String) {
					try {
						importance = Integer.parseInt((String) impObj);
					} catch (NumberFormatException e) {
						getLogger().warn(
								"could not parse importance integer from"
										+ (String) impObj
										+ ". taken default -1.");
						importance = -1;
					}
				}
				int deadline = -1;
				if (impObj != null && impObj instanceof String) {
					try {
						deadline = Integer.parseInt((String) deadlineObj);
					} catch (NumberFormatException e) {
						getLogger().warn(
								"could not parse deadline integer from"
										+ (String) impObj
										+ ". taken default -1.");
						deadline = -1;
					}
				}
				Goal g = new Goal(importance, deadline, (String) goalObj, false);
				goals.add(g);
			}
		}

		try {
			WMEntryQueueElement<PlanningTask> res = planner.plan(goals,
					shouldExecutePlan).get();
			if (res == null)
				return "PLANNING FAILED";
			else {
				if (shouldExecutePlan) {
					addToWorkingMemory(newDataID(), new PlanProxy(res
							.getEvent().address));
				}
				String resultString = res.getEntry().planningStatus.name()
						+ " costs=" + res.getEntry().costs + ", goals:";
				for (Goal g : res.getEntry().goals) {
					resultString += " [" + g.goalString + ", " + g.isInPlan
							+ "]";
				}
				return resultString;
			}
		} catch (Exception e) {
			logException(e);
			return e.getMessage();
		}
	}

}
