/**
 * 
 */
package planning.examples;

import planning.autogen.Planner.ObjectDeclaration;
import planning.components.abstr.PlanningStateGenerator;
import planning.ontology.PlanningOntology;
import planning.ontology.PlanningOntologyFactory;
import planning.util.PlanningDataTranslator;
import planning.util.TemporaryPlanningState;
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
        setOntology(PlanningOntologyFactory.getOntology());
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
        registerPlanStateMapping(
            PlanningOntology.OBJECT_DECLARATION_TYPE, this);
        registerPlanStateMapping(PlanningOntology.FACT_TYPE, this);
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
            if (data.getType().equals(
                PlanningOntology.OBJECT_DECLARATION_TYPE)) {
                state.m_objectList.add((ObjectDeclaration) data
                    .getData());
            }
            else if (data.getType().equals(PlanningOntology.FACT_TYPE)) {
                state.m_factList.add((String) data.getData());
                
            }
//            log(data);
        }

        return state;
    }

}
