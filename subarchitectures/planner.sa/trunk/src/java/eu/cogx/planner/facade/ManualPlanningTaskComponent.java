/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.planner.facade;

import java.util.LinkedList;

import javax.swing.table.TableModel;

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
	ManualPlanningTaskFrame frame;
	WMView<GroundedBelief> view;
	PlannerFacade planner;

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
		frame = new ManualPlanningTaskFrame(model, this);
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
	public String submit(TableModel goalsTable) {
		log("submit:" + goalsTable);
		LinkedList<Goal> goals = new LinkedList<Goal>();
		for (int i=0; i<goalsTable.getRowCount(); i++) {
			Object goalObj=goalsTable.getValueAt(i, 0);
			Object impObj=goalsTable.getValueAt(i, 1);
			if (goalObj!=null && goalObj instanceof String) {
				int importance=-1;
				if (impObj!=null && impObj instanceof String) {
					importance=Integer.parseInt((String) impObj);
				}
				Goal g = new Goal(importance, (String) goalObj, false);
				goals.add(g);
			}
		}
		planner.setGoals(goals);
		try {
			WMEntryQueueElement<PlanningTask> res=planner.call();
			if (res==null)
				return "null";
			else {
				String resultString=res.getEntry().planningStatus.name()+" costs=" + res.getEntry().costs+ ", goals:";
				for (Goal g: res.getEntry().goals) {
					resultString+=" ["+g.goalString + ", " + g.isInPlan +"]";
				}
				return resultString;
			}
		} catch (Exception e) {
			logException(e);
			return e.getMessage();
		}

	}

}
