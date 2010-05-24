/**
 * 
 */
package comsys.fake;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import javax.swing.ComboBoxModel;
import javax.swing.ListModel;




import cast.architecture.ManagedComponent;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.ComsysQueryFeature;
import execution.util.ActionExecutor;
import execution.util.ActionExecutor.ExecutionCompletionCallback;

/**
 * @author marc
 *
 */
public class FakeComsysActionInterface extends ManagedComponent {
	
	class VerificationActionExecutor extends Thread implements ActionExecutor {
	
		String beliefID;
		String featureID;
		
		@Override
		public boolean accept(Action action) {
			ComsysQueryFeature cqf = (ComsysQueryFeature) action;
			beliefID = cqf.beliefID;
			featureID = cqf.featureID;
			return true;
		}
	
		@Override
		public TriBool execute() {
			FakeComsysFrame jFrame = new FakeComsysFrame();
			jFrame.getjLabelQuestion().setText("What's the value of " + featureID + " in belief " + beliefID+ "?");
			
			jFrame.getJButtonOK().addActionListener(new java.awt.event.ActionListener() {
				public void actionPerformed(java.awt.event.ActionEvent e) {
					System.out.println("actionPerformed()"); // TODO Auto-generated Event stub actionPerformed()
				}
			});
			jFrame.getJButtonCancel().addActionListener(new java.awt.event.ActionListener() {
				public void actionPerformed(java.awt.event.ActionEvent e) {
					System.out.println("actionPerformed()"); // TODO Auto-generated Event stub actionPerformed()
				}
			});
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

	final Map<String, List<String> > alternativeValues;
	

	/**
	 * @param alternativeValues
	 */
	public FakeComsysActionInterface() {
		super();
		this.alternativeValues = new HashMap<String, List<String>>();
		
		
		List<String> values;
		
		//rooms
		values=new LinkedList<String>();
		values.add("living_room");
		values.add("office");		
		values.add("dining_room");
		values.add("kitchen");
		alternativeValues.put("Room::label", values);
		
		//objects
		values=new LinkedList<String>();
		values.add("cornflakes");
		values.add("crunchy_nut");		
		values.add("chocopops");
		values.add("miracoli");
		alternativeValues.put("Object::label", values);
		
		//objects
		values=new LinkedList<String>();
		values.add("table");
		values.add("floor");
		alternativeValues.put("Object::position", values);

	}

}
