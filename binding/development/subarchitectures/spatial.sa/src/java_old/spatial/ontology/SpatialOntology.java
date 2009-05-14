/**
 * 
 */
package spatial.ontology;

import spatial.autogen.SpatialLocation;
import spatial.autogen.SpatialRelationshipTargetQuery;
import spatial.autogen.SpatialScene;
import cast.core.ontologies.CASTOntology;

/**
 * @author nah
 */
public class SpatialOntology extends CASTOntology {

    /**
     * 
     */
    SpatialOntology() {
        super();
        addObjectType(SpatialLocation.class);
        addObjectType(SpatialScene.class);
        addObjectType(SpatialRelationshipTargetQuery.class);
    }

}
