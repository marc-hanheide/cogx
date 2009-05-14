/**
 * 
 */
package spatial.components.abstr;

import java.util.Arrays;

import spatial.autogen.SpatialLocation;
import spatial.autogen.SpatialScene;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.data.CASTDataCache;

/**
 * @author nah
 */
public abstract class SpatialSubarchitectureManagedProcess
		extends
			ManagedProcess {

	protected String m_currentSceneID;
	private CASTDataCache<SpatialLocation> m_locationCache;

	/**
	 * @param _id
	 */
	public SpatialSubarchitectureManagedProcess(String _id) {
		super(_id);
		
	}

	private  CASTDataCache<SpatialScene> m_sceneCache;

	/**
	 * @param _currentScene
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	protected SpatialScene getScene(String _currentScene)
			throws SubarchitectureProcessException {
		// CASTTypedData<?> sceneData = getWorkingMemoryEntry(_currentScene);
		// if (sceneData != null) {
		// return (SpatialScene) sceneData.getData();
		// }
		// else {
		// return null;
		// }
		return m_sceneCache.getData(_currentScene);
	}

	/**
	 * Returns the scene after adding this component the list of processes
	 * working on it.
	 * 
	 * @param _sceneID
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	protected SpatialScene getSceneForProcessing(String _sceneID)
			throws SubarchitectureProcessException {
		lockEntry(_sceneID, WorkingMemoryPermissions.LOCKED_ODR);
		SpatialScene scene = (SpatialScene) getWorkingMemoryEntry(_sceneID)
				.getData();

		String[] newlist = new String[scene.m_activeComponents.length + 1];
		System.arraycopy(scene.m_activeComponents, 0, newlist, 0,
				scene.m_activeComponents.length);
		newlist[newlist.length - 1] = getProcessIdentifier();
		scene.m_activeComponents = newlist;
		overwriteWorkingMemory(_sceneID, scene);
		unlockEntry(_sceneID);

		log("getSceneForProcessing: active components: "
				+ Arrays.toString(scene.m_activeComponents));

		return scene;
	}

	@Override
	public void start() {
		super.start();
		m_sceneCache = new CASTDataCache<SpatialScene>(this, SpatialScene.class);
		m_locationCache = new CASTDataCache<SpatialLocation>(this,
				SpatialLocation.class);
	}
	
	protected void endSceneProcessing(String _sceneID)
			throws SubarchitectureProcessException {
		lockEntry(_sceneID, WorkingMemoryPermissions.LOCKED_ODR);
		SpatialScene scene = (SpatialScene) getWorkingMemoryEntry(_sceneID)
				.getData();

		removeSelfFromActiveList(scene);
		overwriteWorkingMemory(_sceneID, scene);
		unlockEntry(_sceneID);

	}

	protected void removeSelfFromActiveList(SpatialScene scene) {
		String[] newList = new String[scene.m_activeComponents.length - 1];

		int o = 0;
		int n = 0;
		// copy omitted this
		while (o < scene.m_activeComponents.length) {
			if (!scene.m_activeComponents[o].equals(getProcessIdentifier())) {
				newList[n++] = scene.m_activeComponents[o];
			}
			o++;
		}

		scene.m_activeComponents = newList;
		log("removeSelfFromActiveList: active components: "
				+ Arrays.toString(scene.m_activeComponents));
	}

	/**
	 * @param _spatialObjectAddress
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	protected SpatialLocation getSpatialLocation(String _spatialObjectAddress)
			throws SubarchitectureProcessException {
		return m_locationCache.getData(_spatialObjectAddress);
	}

	// /**
	// * @param _binding
	// * @param _subarchID
	// * @return
	// * @throws SubarchitectureProcessException
	// */
	// protected WorkingMemoryAddress getSubarchitectureBinding(
	// InstanceBinding _binding, String _subarchID)
	// throws SubarchitectureProcessException {
	// WorkingMemoryAddress[] ecs = _binding.m_bindings;
	// for (int i = 0; i < ecs.length; i++) {
	// // log("subarch check: "+ ecs[i].m_subarchitecture + " == "
	// // + _subarchID);
	// WorkingMemoryAddress candidateAddress = ecs[i];
	//
	// // get the candidate
	// InstanceBindingCandidate candidate = (InstanceBindingCandidate)
	// getWorkingMemoryEntry(
	// candidateAddress.m_subarchitecture,
	// candidateAddress.m_id).getData();
	//
	// if (candidate.m_candidateAddress.m_subarchitecture
	// .equals(_subarchID)) {
	// return candidate.m_candidateAddress;
	// }
	// }
	// return null;
	// }

	// /**
	// * @param _currentScene
	// * @param _wma
	// * @return
	// */
	// protected static boolean sceneContains(SpatialScene _currentScene,
	// WorkingMemoryAddress _wma) {
	// for (int i = 0; i < _currentScene.m_locations.length; i++) {
	// if (CASTUtils.equals(_currentScene.m_locations[i], _wma)) {
	// return true;
	// }
	// }
	// return false;
	// }

}
