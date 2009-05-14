/**
 * 
 */
package manipulation.ontology;

import manipulation.autogen.Manipulation.*;
import cast.core.ontologies.CASTOntology;

/**
 * @author sxh
 */
public class ManipulationOntology extends CASTOntology {

    public static final String PICK_AND_PLACE_COMMAND_TYPE  = PICK_AND_PLACE_CMD_TYPE_STRING.value;
    public static final String  STOP_CMD_TYPE = STOP_CMD_TYPE_STRING.value;
    public static final String  PAUSE_CMD_TYPE = PAUSE_CMD_TYPE_STRING.value ;
    public static final String  RESUME_CMD_TYPE  = RESUME_CMD_TYPE_STRING.value ;
//    public static final String  CMD_COMPLETE_TYPE =  CMD_COMPLETE_TYPE_STRING.value;

    
    /**
     * Create a new instance.
     */
    ManipulationOntology() {
        establishObjectMapping(PickAndPlaceCmd.class,
            PICK_AND_PLACE_COMMAND_TYPE);
        establishObjectMapping(StopCmd.class,
                STOP_CMD_TYPE);
        establishObjectMapping(PauseCmd.class,
                PAUSE_CMD_TYPE);
        establishObjectMapping(ResumeCmd.class,
                RESUME_CMD_TYPE);
//        establishObjectMapping(Comman,
//                CMD_COMPLETE_TYPE);
            
        
    }

}
