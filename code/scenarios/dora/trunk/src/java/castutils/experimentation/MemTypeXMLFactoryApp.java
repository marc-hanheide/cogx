/**
 * 
 */
package castutils.experimentation;

import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.wmeditor.serializer.XMLSerializer;
import execution.slice.ActionStatus;
import execution.slice.TriBool;
import execution.slice.actions.AskForLabelExistence;

/**
 * @author marc
 *
 */
public class MemTypeXMLFactoryApp {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		XMLSerializer s = new XMLSerializer();
		AskForLabelExistence a= new AskForLabelExistence(ActionStatus.PENDING, TriBool.TRIINDETERMINATE, new WorkingMemoryAddress("id", "sa"), new WorkingMemoryAddress("id", "sa"), "label", "relation");
		System.out.println(s.dump(a));
		System.exit(0);
	}

}
