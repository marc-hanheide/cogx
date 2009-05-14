/**
 * 
 */
package spatial.components.abstr;

import spatial.autogen.SpatialLocation;
import spatial.autogen.SpatialScene;
import spatial.ontology.SpatialOntologyFactory;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.core.data.CASTTypedData;

/**
 * @author nah
 */
public abstract class SpatialSubarchitectureManagedProcess
        extends
            ManagedProcess {

    protected String m_currentSceneID;

    /**
     * @param _id
     */
    public SpatialSubarchitectureManagedProcess(String _id) {
        super(_id);
    }

    /**
     * @param _currentScene
     * @return
     * @throws SubarchitectureProcessException
     */
    protected SpatialScene getScene(String _currentScene)
            throws SubarchitectureProcessException {
        CASTTypedData<?> sceneData = getWorkingMemoryEntry(_currentScene);
        if (sceneData != null) {
            return (SpatialScene) sceneData.getData();
        }
        else {
            return null;
        }
    }

    /**
     * @param _spatialObjectAddress
     * @return
     * @throws SubarchitectureProcessException
     */
    protected SpatialLocation getSpatialObject(
            String _spatialObjectAddress)
            throws SubarchitectureProcessException {
        CASTTypedData<?> objectData = getWorkingMemoryEntry(_spatialObjectAddress);
        if (objectData != null) {
            return (SpatialLocation) objectData.getData();
        }
        else {
            return null;
        }
    }

//    /**
//     * @param _binding
//     * @param _subarchID
//     * @return
//     * @throws SubarchitectureProcessException
//     */
//    protected WorkingMemoryAddress getSubarchitectureBinding(
//            InstanceBinding _binding, String _subarchID)
//            throws SubarchitectureProcessException {
//        WorkingMemoryAddress[] ecs = _binding.m_bindings;
//        for (int i = 0; i < ecs.length; i++) {
//            // log("subarch check: "+ ecs[i].m_subarchitecture + " == "
//            // + _subarchID);
//            WorkingMemoryAddress candidateAddress = ecs[i];
//
//            // get the candidate
//            InstanceBindingCandidate candidate = (InstanceBindingCandidate) getWorkingMemoryEntry(
//                candidateAddress.m_subarchitecture,
//                candidateAddress.m_id).getData();
//
//            if (candidate.m_candidateAddress.m_subarchitecture
//                .equals(_subarchID)) {
//                return candidate.m_candidateAddress;
//            }
//        }
//        return null;
//    }

//    /**
//     * @param _currentScene
//     * @param _wma
//     * @return
//     */
//    protected static boolean sceneContains(SpatialScene _currentScene,
//            WorkingMemoryAddress _wma) {
//        for (int i = 0; i < _currentScene.m_locations.length; i++) {
//            if (CASTUtils.equals(_currentScene.m_locations[i], _wma)) {
//                return true;
//            }
//        }
//        return false;
//    }

}
