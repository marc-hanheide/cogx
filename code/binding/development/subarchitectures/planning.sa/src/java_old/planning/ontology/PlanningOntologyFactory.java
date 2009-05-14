/**
 * 
 */
package planning.ontology;

import cast.core.ontologies.CASTOntology;

/**
 * @author nah
 */
public class PlanningOntologyFactory {

    private static CASTOntology m_ontology;

    /**
     * 
     */
    public PlanningOntologyFactory() {
        m_ontology = null;
    }

    public static CASTOntology getOntology() {
        if (m_ontology == null) {
            m_ontology = new PlanningOntology();
        }
        return m_ontology;
    }

}
