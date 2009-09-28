/**
 * 
 */
package motivation.util;

import motivation.slice.CategorizePlaceMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import binder.autogen.core.OriginMap;
import binder.autogen.core.Union;

/**
 * @author marc
 * 
 */
public class GoalTranslator {

	public static String motive2PlannerGoal(HomingMotive m) {
		// TODO: this has to be implemented with lookup to the unions
		return new String ("");
	}
	
	
	public static String motive2PlannerGoal(ExploreMotive m) {
		String placeStr=Long.toString(m.placeID);
		return "(exists (?p - place)  (and (= (place_id ?p) place_id_"+placeStr+") (= (explored ?p) true)))";
		//return new String ("(explored place_id_" + m.placeID+")");
	}


	public static String motive2PlannerGoal(CategorizePlaceMotive m, Union union) {
		String placeStr=Long.toString(m.placeID);
		return "(exists (?p - place)  (and (= (place_id ?p) place_id_"+placeStr+") (kval '" + union.entityID +"' (place_category ?p))))";
		
	}
}
