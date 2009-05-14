/**
 * 
 */
package spatial.components.abstr;

import java.util.Properties;
import java.util.Stack;

import org.cognitivesystems.common.autogen.Math.Vector3D;
import org.cognitivesystems.common.autogen.Math.Vector3DWithConfidence;
import org.cognitivesystems.spatial.pf.PotentialFieldPoint;
import org.cognitivesystems.spatial.pf.ProjectiveMap;
import org.cognitivesystems.spatial.pf.SpatialMapException;

import spatial.autogen.SpatialLocation;
import spatial.autogen.SpatialRelationship;
import spatial.autogen.SpatialRelationshipTargetQuery;
import spatial.autogen.SpatialRelationshipType;
import spatial.autogen.SpatialScene;
import spatial.util.SpatialSubarchitectureException;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

/**
 * Component to manage the structs that relate to the current scene.
 * 
 * @author nah
 */
public abstract class ProjectiveComponent
		extends
			SpatialSubarchitectureManagedProcess {

	public static final float SCENE_BLX = -0.5f;
	public static final float SCENE_BLY = -0.5f;
	public static final float SCENE_TRX = 0.5f;
	public static final float SCENE_TRY = 0.5f;

	private SpatialScene m_currentScene;

	private String m_currentWriteTaskID;

	private Stack<SpatialRelationship> m_relationshipStack;
	private Stack<SpatialRelationship> m_relationshipsToWrite;

	protected ProjectiveMap m_projectiveMap;

	protected float m_projectiveThreshold;

	private String m_lastSceneID;

	/**
	 * @param _id
	 */
	public ProjectiveComponent(String _id) {
		super(_id);

		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

		m_projectiveMap = null;
		m_relationshipStack = new Stack<SpatialRelationship>();
		// Default threshold, set with cmd line -t
		m_projectiveThreshold = 0.8f;

		m_currentWriteTaskID = null;
		m_relationshipsToWrite = null;
	}

	/**
	 * @param _rel
	 * @throws SubarchitectureProcessException
	 */
	private void addRelationshipToCurrentScene(SpatialRelationship _rel)
			throws SubarchitectureProcessException {

		if (m_currentScene == null) {
			throw new SpatialSubarchitectureException("Current scene is null!");
		}

		if (m_currentSceneID == null) {
			throw new SpatialSubarchitectureException(
					"Current scene ID is null!");
		}

		if (m_currentWriteTaskID != null) {
			throw new SpatialSubarchitectureException(
					"Already writing relationships for task: "
							+ m_currentWriteTaskID);
		}

		if (m_relationshipsToWrite == null) {
			m_relationshipsToWrite = new Stack<SpatialRelationship>();
		}
		m_relationshipsToWrite.push(_rel);

		log("extending scene with: " + _rel.m_target + " " + _rel.m_landmark
				+ " " + _rel.m_value);

	}

	/**
	 * @param _curretScene
	 * @param _projectiveMap
	 * @throws SubarchitectureProcessException
	 */
	private void addSceneObjectsToMap(SpatialScene _curretScene,
			ProjectiveMap _projectiveMap)
			throws SubarchitectureProcessException {

		String soID;
		SpatialLocation so;
		// add all the objects into the map
		for (int i = 0; i < _curretScene.m_locations.length; i++) {
			// get spatial object from WM
			soID = _curretScene.m_locations[i];
			so = getSpatialObject(soID);
			// add to map

			try {
				_projectiveMap.addObject(soID, so.m_centroid.m_x,
						so.m_centroid.m_y, so.m_centroid.m_z);
			}
			catch (SpatialMapException e) {
				log("Ignoring object beyond map boundaries: " + soID);
			}
		}

	}

	/**
	 * Create a list of all the relations that must be checked in the map
	 * 
	 * @param _currentScene
	 */
	private void buildRelationshipStack(SpatialScene _currentScene) {
		String[] objectIDs = _currentScene.m_locations;
		m_relationshipStack.clear();

		String target;
		String landmark;
		for (int i = 0; i < objectIDs.length; i++) {
			target = objectIDs[i];
			for (int j = 0; j < objectIDs.length; j++) {
				landmark = objectIDs[j];
				if (!target.equals(landmark)) {
					m_relationshipStack.push(new SpatialRelationship(target,
							landmark, 0));
				}
			}
		}
	}

	/**
	 * @param _address
	 * @return
	 * @throws SubarchitectureProcessException
	 */
	@SuppressWarnings("unchecked")
	protected CASTData<SpatialRelationshipTargetQuery> getQuery(
			WorkingMemoryAddress _address)
			throws SubarchitectureProcessException {
		CASTData data = getWorkingMemoryEntry(_address.m_id,
				_address.m_subarchitecture);
		if (data.getData() instanceof SpatialRelationshipTargetQuery) {
			return data;
		}

		throw new SpatialSubarchitectureException(
				"address points to non query object: "
						+ CASTUtils.toString(_address));
	}

	/**
	 * @return
	 */
	private boolean haveRelationshipsToWrite() {
		return m_relationshipsToWrite != null;
	}

	/**
	 * @param _sceneID
	 */
	private void newScene(String _sceneID) {

		try {
			if (m_currentSceneID != null) {
				println("!!!!!!!!!!!!!!scene processing interrupted!!!!!!!!!!");
			}

			m_currentSceneID = _sceneID;

			// used to store scene for query processing
			m_lastSceneID = _sceneID;

			// get the current scene object
			m_currentScene = getScene(m_currentSceneID);
			// remove projective map
			m_projectiveMap = null;

			// empty writing relationship stack
			m_relationshipsToWrite = null;
			// reset write task id
			m_currentWriteTaskID = null;
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @param _rel
	 * @throws SubarchitectureProcessException
	 */
	private void processRelationship(SpatialRelationship _rel)
			throws SubarchitectureProcessException {

		if (m_projectiveMap != null) {
			float proxVal = m_projectiveMap.projectiveValue(_rel.m_target,
					_rel.m_landmark);

			_rel.m_value = proxVal;

			if (proxVal > m_projectiveThreshold) {
				// println("adding: " + _rel.m_target + " "
				// + _rel.m_landmark + " " + _rel.m_value);

				addRelationshipToCurrentScene(_rel);
			}
			else {
				// println("not adding: " + _rel.m_target + " "
				// + _rel.m_landmark + " " + _rel.m_value);
			}

		}
		else {
			throw new SpatialSubarchitectureException("projective map is null");
		}

	}

	/**
	 * @param _response
	 * @throws SubarchitectureProcessException
	 */
	private void respondToQuery(
			CASTData<SpatialRelationshipTargetQuery> _queryData)
			throws SubarchitectureProcessException {

		SpatialRelationshipTargetQuery query = _queryData.getData();

		if (m_lastSceneID != null) {
			// testing
			// _response.m_targets = new Vector3DWithConfidence[0];

			if (m_currentSceneID != null) {
				// this should never happe
				throw new SpatialSubarchitectureException(
						"current scene still active");
			}

			// HACK ... this isn't nicely designed!!!
			// construct a map

			// HACK ... and it's only using the last scene for
			// robustness

			// use current scene if not given a scene
			if (query.m_sceneAddress.m_id.length() == 0) {
				log("using current scene for rel query");
				m_currentSceneID = m_lastSceneID;
			}
			// otherwise generate a new map from the given scene
			else {
				m_currentSceneID = query.m_sceneAddress.m_id;
				m_currentScene = getScene(m_currentSceneID);
				constructSceneMap();
			}

			m_projectiveMap.projectiveMap(query.m_landmarkID);

			// at this point we can get the sweet spots

			PotentialFieldPoint pfp;
			query.m_targets = new Vector3DWithConfidence[query.m_requestedTargets];
			for (int i = 0; i < query.m_requestedTargets; i++) {

				pfp = m_projectiveMap.nextSweetSpot();

				log("sweet spot: " + i + " " + pfp.getX() + " " + pfp.getY()
						+ " " + pfp.getValue());

				query.m_targets[i] = new Vector3DWithConfidence(new Vector3D(
						pfp.getX(), pfp.getY(), 0), pfp.getValue());
			}

			// reset();
			m_currentSceneID = null;
		}
		else {
			log("no scene to process");
			query.m_targets = new Vector3DWithConfidence[0];
		}

		log("written response");

		// send response
		overwriteWorkingMemory(_queryData.getID(), query);
	}

	/**
	 * 
	 */
	private void reset() {
		m_currentSceneID = null;
		m_currentScene = null;
		m_projectiveMap = null;
	}

	private void writeRelationshipsToWorkingMemory()
			throws SubarchitectureProcessException {

		assert (m_currentSceneID != null);

		// lock while we're doing this to prevent concurrency errors
		lockEntry(m_currentSceneID, WorkingMemoryPermissions.LOCKED_ODR);

		// get the latest version of the current scene
		m_currentScene = getScene(m_currentSceneID);

		setRelationships(m_currentScene,
				(SpatialRelationship[]) m_relationshipsToWrite
						.toArray(new SpatialRelationship[m_relationshipsToWrite
								.size()]));

		log("extending scene with: " + m_relationshipsToWrite.size()
				+ " relationships");

		// and overwrite in working memory
		overwriteWorkingMemory(m_currentSceneID, m_currentScene);
		unlockEntry(m_currentSceneID);
	}

	/**
	 * @return
	 */
	protected abstract String getProcessGoalString();

	protected abstract SpatialRelationship[] getRelationships(
			SpatialScene _scene);

	protected abstract String getWriteGoalString();

	/**
	 * 
	 */
	protected abstract ProjectiveMap newProjectiveMap();

	// /*
	// * (non-Javadoc)
	// *
	// * @see cast.architecture.subarchitecture.ManagedProcess#runComponent()
	// */
	// @Override
	// protected void runComponent() {

	// while (m_status == ProcessStatus.RUN) {

	// try {
	// if (m_currentScene != null) {
	//
	// if (m_projectiveMap == null) {
	// // println("constructing projective map");
	//
	// constructSceneMap();
	//
	// buildRelationshipQueue(m_currentScene);
	//
	// proposeRelationshipProcessing();
	// }
	//
	// // now calc projective relations between all objects
	// // in map
	//
	// if (m_processingAllowed) {
	// // keep going through rels
	// if (!m_relationshipStack.empty()) {
	// processRelationship(m_relationshipStack.pop());
	// }
	// else {
	// // we're done!
	// taskComplete(m_currentProcessTaskID,
	// TaskOutcome.PROCESSING_COMPLETE_SUCCESS);
	// m_processingAllowed = false;
	//
	// if (haveRelationshipsToWrite()) {
	// // now write results to wm
	// proposeRelationshipWriting();
	// }
	// else {
	// // forget everything
	// m_currentSceneID = null;
	// }
	// }
	// }
	// }
	//
	// // only sleep if there's nothing to be done now
	// if (m_relationshipStack.empty()) {
	// // if there are queries to respond to
	// if (m_queryQueue.size() > 0) {
	// CASTData<SpatialRelationshipTargetQuery> query = m_queryQueue
	// .remove();
	// respondToQuery(query);
	// }
	// else {
	// sleepProcess(500);
	// }
	// }
	//
	// }
	// catch (SubarchitectureProcessException e) {
	// e.printStackTrace();
	//
	// // reset scene
	// m_currentSceneID = null;
	// m_currentScene = null;
	// m_projectiveMap = null;
	// m_processingAllowed = false;
	// }

	// }
	//
	// }

	/**
	 * @throws SubarchitectureProcessException
	 */
	private void constructSceneMap() throws SubarchitectureProcessException {
		// HACK where do these numbers come from?
		// scene is 1m square with 0,0 at the centre
		m_projectiveMap = newProjectiveMap();

		// viewer is out of the scene at 0.0, -1.0, but
		// that shouldn't matter for proximal at least
		m_projectiveMap.setViewerPosition(
				m_currentScene.m_viewerPose.m_position.m_x,
				m_currentScene.m_viewerPose.m_position.m_y,
				m_currentScene.m_viewerPose.m_position.m_z);

		addSceneObjectsToMap(m_currentScene, m_projectiveMap);
	}

	protected abstract void setRelationships(SpatialScene _scene,
			SpatialRelationship[] _rels);

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _goalID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _goalID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.CAST.WorkingMemoryChange[])
	 */
	@Override
	public void start() {

		super.start();

		try {
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					SpatialScene.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							newScene(_wmc.m_address.m_id);
							processScene();
							reset();
						}
					});

			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					SpatialRelationshipTargetQuery.class,
					WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {

							try {
								log("seen query type!!!");

								CASTData<SpatialRelationshipTargetQuery> query = getQuery(_wmc.m_address);

								// if it's our type
								if (isMapType(query.getData().m_rel)) {
									respondToQuery(query);
								}
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
							}

						}
					});
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

	private void processScene() {
		try {
			if (m_projectiveMap == null) {
				// println("constructing projective map");
				constructSceneMap();
				buildRelationshipStack(m_currentScene);
			}

			// now calc projective relations between all objects
			// in map

			// keep going through rels
			while (!m_relationshipStack.empty()) {
				processRelationship(m_relationshipStack.pop());
			}

			if (haveRelationshipsToWrite()) {
				// now write results to wm
				writeRelationshipsToWorkingMemory();
			}
		}
		catch (SubarchitectureProcessException e) {
			println(e);
		}
	}
	/**
	 * @param _rel
	 * @return
	 */
	protected abstract boolean isMapType(SpatialRelationshipType _rel);

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config) {
		super.configure(_config);
		if (_config.containsKey("-t")) {
			m_projectiveThreshold = Float.parseFloat(_config.getProperty("-t"));
		}
	}

}
