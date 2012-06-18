/**
 * 
 */
package castutils.experimentation;

import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.wmeditor.serializer.XMLSerializer;
import execution.slice.ActionStatus;
import execution.slice.TriBool;
import execution.slice.actions.AskForBKLabelInCategory;
import execution.slice.actions.AskForBKLabelRelLabel;
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
		AskForBKLabelInCategory b= new AskForBKLabelInCategory(ActionStatus.PENDING, TriBool.TRIINDETERMINATE, "magazine", "kitchen");
		AskForBKLabelRelLabel c= new AskForBKLabelRelLabel(ActionStatus.PENDING, TriBool.TRIINDETERMINATE,  "magazine", "in", "box");
		Place p = new Place(4, PlaceStatus.TRUEPLACE);
		System.out.println(s.dump(p));
		System.exit(0);
	}

}
