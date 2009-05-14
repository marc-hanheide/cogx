/**
 * 
 */
package planning.examples;

import planning.components.abstr.PlanningStateGenerator;
import planning.util.PlanningDataTranslator;
import planning.util.PlanningUtils;
import planning.util.TemporaryPlanningState;
import Planner.Fact;
import Planner.ObjectDeclaration;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

/**
 * Generate a planning state from a working memory that contains just
 * planning-related objects.
 * 
 * @author nah
 */
public class SimpleStateGenerator extends PlanningStateGenerator
        implements PlanningDataTranslator {

    /**
     * @param _id
     */
    public SimpleStateGenerator(String _id) {
        super(_id);
    }

    /*
     * (non-Javadoc)
     * 
     * @see planning.components.abstr.PlanningStateGenerator#start()
     */
    @Override
    public void start() {
        super.start();
        // do this here to ensure that all internal variable have been
        // set
        registerPlanStateMapping(ObjectDeclaration.class, this);
        registerPlanStateMapping(Fact.class, this);
    }

    /*
     * (non-Javadoc)
     * 
     * @see planning.util.PlanningDataTranslator#toPlanningState(cast.core.data.CASTData<?>[])
     */
    public TemporaryPlanningState toPlanningState(CASTData<?>[] _wme) {
        TemporaryPlanningState state = new TemporaryPlanningState();
        for (CASTData<?> data : _wme) {
            // println(data);
            // just a simple copy of what's in working memory
            if (data.getType().equals(CASTUtils.typeName(ObjectDeclaration.class))) {
                state.m_objectList.add((ObjectDeclaration) data
                    .getData());
            }
            else if (data.getType().equals(CASTUtils.typeName(Fact.class))) {
                state.m_factList.add((Fact) data.getData());
            }
//            log(data);
        }

        return state;
    }

}
