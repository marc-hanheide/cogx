/**
 * 
 */
package spatial.ontology;


import vision.ontology.VisionOntologyFactory;
import binding.ontology.BindingOntologyFactory;
import cast.core.ontologies.CASTCompositeOntology;
import cast.core.ontologies.CASTOntology;

/**
 * @author nah
 */
public class SpatialOntologyFactory {

    private static CASTCompositeOntology m_compositeOntology;

    /**
     * 
     */
    public SpatialOntologyFactory() {
        super();
        m_compositeOntology = null;
    }


    public static CASTOntology getOntology() {

        if (m_compositeOntology == null) {
            
            m_compositeOntology = new CASTCompositeOntology();

            m_compositeOntology.addOntology(VisionOntologyFactory
                .getOntology());

            m_compositeOntology.addOntology(new SpatialOntology());

            m_compositeOntology.addOntology(BindingOntologyFactory
                .getOntology());
        }

        return m_compositeOntology;
    }

}
