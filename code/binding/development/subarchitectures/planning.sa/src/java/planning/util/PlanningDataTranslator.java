/**
 * 
 */
package planning.util;

import cast.core.data.CASTData;

/**
 * A class that translates between a list of working memory entries and
 * an arbitrary number of object declarations and facts in a planning
 * state.
 * 
 * @author nah
 */
public interface PlanningDataTranslator {

    /**
     * The methods receives a list of entries from working memory and
     * must return them as a planning state.
     * 
     * @param _wme
     * @return
     */
    TemporaryPlanningState toPlanningState(CASTData<?>[] _wme);

}
