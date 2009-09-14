/**
 * 
 */
package motivation.util;

import motivation.slice.ExploreMotive;
import motivation.slice.Motive;

/**
 * @author marc
 * 
 */
public class GoalTranslator<M extends Motive> {
	public String motive2PlannerGoal(M motive) {
		return "()";

	}
	public String motive2PlannerGoal(ExploreMotive m) {
		return new String ("(Explore " + Long.toString(m.placeID) +")");
	
	}
}
