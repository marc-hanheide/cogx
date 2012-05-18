/**
 * 
 */
package motivation.components.filters;

import motivation.slice.ExploreMotive;
import motivation.slice.GeneralGoalMotive;
import motivation.slice.HomingMotive;
import motivation.slice.MotivePriority;
import motivation.slice.PatrolMotive;

/**
 * @author marc
 *
 */
public class DoraManualMotiveFilter extends AbstractManualSelectFilter {
	private static final String EXTERNAL_GOAL = "External Goal";
	private static final String DAM_DEFAULT = "DAM default";

	@Override
	protected void registerTypes() {
		addDefault(EXTERNAL_GOAL, GeneralGoalMotive.class, MotivePriority.HIGH);
		addDefault(STARTUP_PRIORITIES, GeneralGoalMotive.class, MotivePriority.HIGH);
		
		
		addDefault(DAM_DEFAULT, ExploreMotive.class, MotivePriority.NORMAL);
		
		addDefault(DAM_DEFAULT, HomingMotive.class, MotivePriority.UNSURFACE);
		addDefault(DAM_DEFAULT, PatrolMotive.class, MotivePriority.HIGH);


		
	}

	public static void main(String[] args) {
		AbstractManualSelectFilter o = new DoraManualMotiveFilter();
		o.start();
	}

}
