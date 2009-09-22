/**
 * 
 */
package motivation.util;

import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import binder.autogen.core.OriginMap;

/**
 * @author marc
 * 
 */
public class GoalTranslator {

	public static String motive2PlannerGoal(HomingMotive m, OriginMap om) {
		// TODO: this has to be implemented with lookup to the unions
		return new String ("");
	}
	
	
	public static String motive2PlannerGoal(ExploreMotive m, OriginMap om) {
		String placeStr=Long.toString(m.placeID);
		return "(exists (?p - place)  (and (= (place_id ?p) place_id_"+placeStr+") (= (explored ?p) true)))";
		//return new String ("(explored place_id_" + m.placeID+")");
	}
}
