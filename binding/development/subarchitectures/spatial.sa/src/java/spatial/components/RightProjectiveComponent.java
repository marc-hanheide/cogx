/**
 * 
 */
package spatial.components;

import org.cognitivesystems.spatial.pf.ProjectiveMap;
import org.cognitivesystems.spatial.pf.RightProjectiveMap;

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
public class RightProjectiveComponent extends ProjectiveComponent {

    /**
     * @param _id
     */
    public RightProjectiveComponent(String _id) {
        super(_id);
    }

    protected SpatialRelationship[] getRelationships(SpatialScene _scene) {
        return _scene.m_rightRelationships;
    }

    protected void setRelationships(SpatialScene _scene,
            SpatialRelationship[] _rels) {
        _scene.m_rightRelationships = _rels;
    }

    /**
     * 
     */
    protected ProjectiveMap newProjectiveMap() {
        return new RightProjectiveMap(-1.0f, -1.0f, 1.0f, 1.0f);
    }

    /*
     * (non-Javadoc)
     * 
     * @see org.cognitivesystems.spatial.components.ProjectiveComponent#getGoalString()
     */
    @Override
    protected String getProcessGoalString() {
        return SpatialGoals.PROCESS_RIGHT_PROJ;
    }

    protected String getWriteGoalString() {
        return SpatialGoals.WRITE_RIGHT_PROJ;
    }

    
    /* (non-Javadoc)
     * @see org.cognitivesystems.spatial.components.abstr.ProjectiveComponent#isMapType(org.cognitivesystems.spatial.autogen.SpatialData.SpatialRelationshipType)
     */
    @Override
    protected boolean isMapType(SpatialRelationshipType _rel) {
        return _rel == SpatialRelationshipType.SPATIAL_RIGHT;
    } 
}
