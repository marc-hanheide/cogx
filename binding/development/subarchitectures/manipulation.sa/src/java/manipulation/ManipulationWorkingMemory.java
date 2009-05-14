/**
 * 
 */
package manipulation;

import manipulation.ontology.ManipulationOntologyFactory;
import planning.ontology.PlanningOntologyFactory;
import visionarch.global.VisionOntologyFactory;
import binding.ontology.BindingOntologyFactory;
import cast.architecture.subarchitecture.SubarchitectureWorkingMemory;
import cast.core.ontologies.CASTCompositeOntology;


/**
 * @author nah
 *
 */
public class ManipulationWorkingMemory
        extends
            SubarchitectureWorkingMemory {

    /**
     * @param _id
     */
    public ManipulationWorkingMemory(String _id) {
        super(_id);
        CASTCompositeOntology ontology = new CASTCompositeOntology();
        ontology.addOntology(BindingOntologyFactory.getOntology());
        ontology.addOntology(PlanningOntologyFactory.getOntology());
        ontology.addOntology(ManipulationOntologyFactory.getOntology());
        ontology.addOntology(VisionOntologyFactory.getOntology());
        setSendXarchChangeNotifications(true);
    }

}
