/**
 * 
 */
package comsys.fake;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.StringValue;

import cast.CASTException;
import cast.architecture.ManagedComponent;
import castutils.facades.BinderFacade;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.ComsysQueryFeature;
import execution.util.ActionExecutor;

/**
 * @author marc
 * 
 */
public class FakeComsysActionInterface extends ManagedComponent {

	public boolean finished;

	class VerificationActionExecutor extends Thread implements ActionExecutor {

		/**
		 * @param comp
		 */
		public VerificationActionExecutor(ManagedComponent comp) {
			super();
			this.comp = comp;
		}

		Belief belief;
		String featureID;
		ManagedComponent comp;

		@Override
		public boolean accept(Action action) {
			ComsysQueryFeature cqf = (ComsysQueryFeature) action;
			belief = BinderFacade.get(comp).getBelief(cqf.beliefID);
			featureID = cqf.featureID;
			return true;
		}

		@Override
		public TriBool execute() {
			FakeComsysFrame jFrame = new FakeComsysFrame();
			List<FeatureValue> fvs = BinderFacade.get(comp).getFeatureValue(
					belief, featureID);
			if (fvs.size() > 0 && fvs.get(0) instanceof StringValue) {
				jFrame.getjLabelQuestion().setText(
						"What's the value of " + featureID + " in belief "
								+ belief.id + "(" + belief.type
								+ ")? Currently it is "
								+ ((StringValue) fvs.get(0)).val);
			} else {
				jFrame.getjLabelQuestion().setText(
						"What's the value of " + featureID + " in belief "
								+ belief.id + "(" + belief.type + ")?");
			}

			jFrame.getJButtonOK().addActionListener(
					new java.awt.event.ActionListener() {
						public void actionPerformed(java.awt.event.ActionEvent e) {
							System.out.println("actionPerformed()"); // TODO
							// Auto-generated
							// Event
							// stub
							// actionPerformed()
						}
					});
			jFrame.getJButtonCancel().addActionListener(
					new java.awt.event.ActionListener() {
						public void actionPerformed(java.awt.event.ActionEvent e) {
							System.out.println("actionPerformed()"); // TODO
							// Auto-generated
							// Event
							// stub
							// actionPerformed()
						}
					});
			jFrame.setVisible(true);
			try {
			while (!finished) {
				try {
					wait();
				} catch (InterruptedException e1) {
					return TriBool.TRIINDETERMINATE;
				}
			} 
			} finally {
				jFrame.setVisible(false);
				jFrame.dispose();
			}
			
			return TriBool.TRITRUE;
		}

		@Override
		public void execute(ExecutionCompletionCallback callback) {
		}

		@Override
		public boolean isBlockingAction() {
			// TODO Auto-generated method stub
			return true;
		}

		@Override
		public void stopExecution() {
			// TODO Auto-generated method stub

		}

	}

	final Map<String, List<String>> alternativeValues;

	/**
	 * @param alternativeValues
	 */
	public FakeComsysActionInterface() {
		super();
		this.alternativeValues = new HashMap<String, List<String>>();

		List<String> values;

		// rooms
		values = new LinkedList<String>();
		values.add("living_room");
		values.add("office");
		values.add("dining_room");
		values.add("kitchen");
		alternativeValues.put("Room::label", values);

		// objects
		values = new LinkedList<String>();
		values.add("cornflakes");
		values.add("crunchy_nut");
		values.add("chocopops");
		values.add("miracoli");
		alternativeValues.put("Object::label", values);

		// objects
		values = new LinkedList<String>();
		values.add("table");
		values.add("floor");
		alternativeValues.put("Object::position", values);

	}

}
