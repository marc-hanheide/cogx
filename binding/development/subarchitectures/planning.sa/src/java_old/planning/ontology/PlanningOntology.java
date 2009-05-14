/**
 * 
 */
package planning.ontology;

import planning.autogen.Planner.ObjectDeclaration;
import planning.autogen.PlanningData.ACTION_REGISTRATION_TYPE_STRING;
import planning.autogen.PlanningData.ACTION_TYPE_STRING;
import planning.autogen.PlanningData.Action;
import planning.autogen.PlanningData.ActionRegistration;
import planning.autogen.PlanningData.FACT_TYPE_STRING;
import planning.autogen.PlanningData.OBJECT_DECLARATION_TYPE_STRING;
import planning.autogen.PlanningData.PLANNING_PROBLEM_TYPE_STRING;
import planning.autogen.PlanningData.PLANNING_PROCESS_REQUEST_TYPE_STRING;
import planning.autogen.PlanningData.PLANNING_STATE_REQUEST_TYPE_STRING;
import planning.autogen.PlanningData.PLAN_STEP_TYPE_STRING;
import planning.autogen.PlanningData.PLAN_TYPE_STRING;
import planning.autogen.PlanningData.Plan;
import planning.autogen.PlanningData.PlanningProblem;
import planning.autogen.PlanningData.PlanningProcessRequest;
import planning.autogen.PlanningData.PlanningStateRequest;
import cast.core.ontologies.CASTOntology;

/**
 * The ontology for planning objects. Any component that wants to
 * manipulate planning structures must include this ontology.
 * 
 * @author nah
 */
public class PlanningOntology extends CASTOntology {

    public static final String PLANNING_PROBLEM_TYPE =
            PLANNING_PROBLEM_TYPE_STRING.value;

    public static final String PLAN_TYPE = PLAN_TYPE_STRING.value;

    public static final String PLANNING_STATE_REQUEST_TYPE =
            PLANNING_STATE_REQUEST_TYPE_STRING.value;

    public static final String PLANNING_PROCESS_REQUEST_TYPE =
            PLANNING_PROCESS_REQUEST_TYPE_STRING.value;

    public static final String ACTION_TYPE = ACTION_TYPE_STRING.value;

    public static final String OBJECT_DECLARATION_TYPE =
            OBJECT_DECLARATION_TYPE_STRING.value;

    public static final String FACT_TYPE = FACT_TYPE_STRING.value;

    public static final String PLAN_STEP_TYPE =
            PLAN_STEP_TYPE_STRING.value;
    
    public static final String ACTION_REGISTRATION_TYPE = ACTION_REGISTRATION_TYPE_STRING.value;

    /**
     * 
     */
    protected PlanningOntology() {
        establishObjectMapping(PlanningProblem.class,
            PLANNING_PROBLEM_TYPE);
        establishObjectMapping(Plan.class, PLAN_TYPE);
        establishObjectMapping(PlanningStateRequest.class,
            PLANNING_STATE_REQUEST_TYPE);
        establishObjectMapping(PlanningProcessRequest.class,
            PLANNING_PROCESS_REQUEST_TYPE   );
        establishObjectMapping(Action.class, ACTION_TYPE);
        establishObjectMapping(ObjectDeclaration.class,
            OBJECT_DECLARATION_TYPE);   
        establishObjectMapping(String.class, FACT_TYPE);
        establishObjectMapping(String.class, PLAN_STEP_TYPE);
        establishObjectMapping(ActionRegistration.class,ACTION_REGISTRATION_TYPE);
    }

}
