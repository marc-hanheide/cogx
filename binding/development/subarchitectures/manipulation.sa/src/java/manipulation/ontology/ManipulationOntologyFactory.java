/**
 * 
 */
package manipulation.ontology;

/**
 * @author sxh
 */
public class ManipulationOntologyFactory {

    private static ManipulationOntology m_ontology;

    static {
        m_ontology = null;
    }

    public static ManipulationOntology getOntology() {
        if (m_ontology == null) {
            m_ontology = new ManipulationOntology();
        }
        return m_ontology;
    }

}
