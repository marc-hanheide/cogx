/**
 * 
 */
package spatial.components;

import org.cognitivesystems.spatial.pf.LeftProjectiveMap;
import org.cognitivesystems.spatial.pf.ProjectiveMap;

import spatial.autogen.SpatialRelationship;
import spatial.autogen.SpatialRelationshipType;
import spatial.autogen.SpatialScene;
import spatial.components.abstr.ProjectiveComponent;
import spatial.util.SpatialGoals;

/**
 * Component to manage the structs that relate to the current scene.
 * 
 * @author nah
 */
public class LeftProjectiveComponent
        extends
            ProjectiveComponent {


    /**
     * @param _id
     */
    public LeftProjectiveComponent(String _id) {
        super(_id);
    }
 
    protected SpatialRelationship[] getRelationships(SpatialScene _scene) {
        return _scene.m_leftRelationships;
    }

    protected void setRelationships(SpatialScene _scene,
            SpatialRelationship[] _rels) {
        _scene.m_leftRelationships = _rels;
    }

    /**
     * 
     */
    protected ProjectiveMap newProjectiveMap() {
        return new LeftProjectiveMap(-1.0f,
            -1.0f, 1.0f, 1.0f);
    }

    /* (non-Javadoc)
     * @see org.cognitivesystems.spatial.components.ProjectiveComponent#getGoalString()
     */
    @Override
    protected String getProcessGoalString() {   
        return SpatialGoals.PROCESS_LEFT_PROJ;
    }
 
    protected String getWriteGoalString() {
        return SpatialGoals.WRITE_LEFT_PROJ;
    }
    
    /* (non-Javadoc)
     * @see org.cognitivesystems.spatial.components.abstr.ProjectiveComponent#isMapType(org.cognitivesystems.spatial.autogen.SpatialData.SpatialRelationshipType)
     */
    @Override
    protected boolean isMapType(SpatialRelationshipType _rel) {
        return _rel == SpatialRelationshipType.SPATIAL_LEFT;
    } 
}
