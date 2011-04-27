/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import javax.swing.table.TableModel;

import motivation.components.generators.ManualGoalFrame.SubmitListener;
import motivation.slice.TutorInitiativeMotive;
import autogen.Planner.Goal;
import cast.AlreadyExistsOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;
import castutils.viewer.WMJTableModel;
import eu.cogx.beliefs.slice.GroundedBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class ManualGoalComponent extends ManagedComponent implements
		SubmitListener {
	ManualGoalFrame frame;
	WMView<GroundedBelief> view;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
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
		frame = new ManualGoalFrame(model, this);
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
		for (int i = 0; i < goalsTable.getRowCount(); i++) {
			Object goalObj = goalsTable.getValueAt(i, 0);
			Object impObj = goalsTable.getValueAt(i, 1);
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
						return "";
					}
				}
				Goal g = new Goal(importance, (String) goalObj, false);
				TutorInitiativeMotive motive = new TutorInitiativeMotive();
				AbstractEpistemicObjectMotiveGenerator.fillDefault(motive);
				motive.thisEntry = new WorkingMemoryAddress(newDataID(),
						getSubarchitectureID());
				motive.goal = g;
				motive.informationGain = 1.0;
				motive.referenceEntry=motive.thisEntry;
				try {
					addToWorkingMemory(motive.thisEntry.id, motive);
				} catch (AlreadyExistsOnWMException e) {
					logException(e);
					return "";
				}
				return "";
			}
		}
		return "";

	}

}
