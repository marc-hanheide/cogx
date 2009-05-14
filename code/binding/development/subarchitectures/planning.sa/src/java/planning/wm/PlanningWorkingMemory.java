/**
 * 
 */
package planning.wm;

import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;

/**
 * A class that can be used as a base class for any working memory that
 * needs to contain planning data. Alternatively, you can just add the
 * PlanningOntology to a composite ontology for your existing working
 * memory.
 * 
 * @author nah
 */
public class PlanningWorkingMemory
        extends
            SubarchitectureWorkingMemory {

    /**
     * @param _id
     */
    public PlanningWorkingMemory(String _id) {
        super(_id);
        setSendXarchChangeNotifications(true);
    }

}
