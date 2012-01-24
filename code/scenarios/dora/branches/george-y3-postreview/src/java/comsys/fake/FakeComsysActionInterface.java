/**
 * 
 */
package comsys.fake;

import cast.architecture.ManagedComponent;

/**
 * @author marc
 *
 */
public class FakeComsysActionInterface extends ManagedComponent {
	
//	class VerificationActionExecutor extends Thread implements ActionExecutor {
//	
//		String beliefID;
//		String featureID;
//		
//		@Override
//		public boolean accept(Action action) {
//			AskForFeatureValue cqf = (AskForFeatureValue) action;
//			beliefID = cqf.beliefID;
//			featureID = cqf.featureID;
//			return true;
//		}
//	
//		@Override
//		public TriBool execute() {
//			FakeComsysFrame jFrame = new FakeComsysFrame();
//			jFrame.getjLabelQuestion().setText("What's the value of " + featureID + " in belief " + beliefID+ "?");
//			
//			jFrame.getJButtonOK().addActionListener(new java.awt.event.ActionListener() {
//				public void actionPerformed(java.awt.event.ActionEvent e) {
//					System.out.println("actionPerformed()"); // TODO Auto-generated Event stub actionPerformed()
//				}
//			});
//			jFrame.getJButtonCancel().addActionListener(new java.awt.event.ActionListener() {
//				public void actionPerformed(java.awt.event.ActionEvent e) {
//					System.out.println("actionPerformed()"); // TODO Auto-generated Event stub actionPerformed()
//				}
//			});
//			return TriBool.TRITRUE;
//		}
//	
//		@Override
//		public void execute(ExecutionCompletionCallback callback) {
//		}
//	
//		@Override
//		public boolean isBlockingAction() {
//			// TODO Auto-generated method stub
//			return true;
//		}
//	
//		@Override
//		public void stopExecution() {
//			// TODO Auto-generated method stub
//			
//		}
//		
//	}
//
//	final Map<String, List<String> > alternativeValues;
//	
//
//	/**
//	 * @param alternativeValues
//	 */
//	public FakeComsysActionInterface() {
//		super();
//		this.alternativeValues = new HashMap<String, List<String>>();
//		
//		
//		List<String> values;
//		
//		//rooms
//		values=new LinkedList<String>();
//		values.add("living_room");
//		values.add("office");		
//		values.add("dining_room");
//		values.add("kitchen");
//		alternativeValues.put("Room::label", values);
//		
//		//objects
//		values=new LinkedList<String>();
//		values.add("cornflakes");
//		values.add("crunchy_nut");		
//		values.add("chocopops");
//		values.add("miracoli");
//		alternativeValues.put("Object::label", values);
//		
//		//objects
//		values=new LinkedList<String>();
//		values.add("table");
//		values.add("floor");
//		alternativeValues.put("Object::position", values);
//
//	}

}
