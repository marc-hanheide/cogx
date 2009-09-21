/**
 * 
 */
package motivation.util;

import binder.autogen.core.OriginMap;
import motivation.slice.ExploreMotive;
import motivation.slice.Motive;

/**
 * @author marc
 * 
 */
public class GoalTranslator {

	public static String motive2PlannerGoal(Motive m, OriginMap om) {
		return new String ("");
	}
	
	
	public static String motive2PlannerGoal(ExploreMotive m, OriginMap om) {
		String placeStr=Long.toString(m.placeID);
		return "(exists (?p - place)  (and (= (place_id ?p) place_id_"+placeStr+") (= (explored ?p) true)))";
		//return new String ("(explored place_id_" + m.placeID+")");
	}
}
