/**
 * 
 */
package spatial.components;

import java.util.ArrayList;
import java.util.Hashtable;

import org.cognitivesystems.common.autogen.Math.Vector3D;

import spatial.autogen.AddToScene;
import spatial.autogen.SpatialLocation;
import spatial.autogen.SpatialRelationship;
import spatial.autogen.SpatialScene;
import spatial.components.abstr.SpatialSubarchitectureManagedProcess;
import spatial.util.SpatialGoals;
import spatial.util.SpatialSubarchitectureException;
import spatial.util.SpatialUtils;
import Vision.Camera;
import Vision.SceneChanged;
import Vision.SceneObject;
import balt.corba.autogen.FrameworkBasics.BALTTime;
import balt.jni.NativeProcessLauncher;
import balt.management.ProcessLauncher;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.FilterRestriction;
import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.cdl.WorkingMemoryPointer;
import cast.cdl.guitypes.FAT;
import cast.core.CASTUtils;
import cast.core.data.CASTTypedData;

/**
 * Component to manage the structs that relate to the current scene. Also adds
 * viewer pose information into the scene struct.
 * 
 * 
 * When the visual scene is static, this component creates a
 * {@link SpatialScene} struct to represent the current static scene. Currently
 * old scenes are not deleted, but they easily could be if things get too nasty.
 * 
 * @author nah
 */
public class SceneManager extends SpatialSubarchitectureManagedProcess {

	/**
	 * A class that monitors a visual object for changes and updates a local
	 * representation of it.
	 * 
	 * @author nah
	 * 
	 */
	private class VisualObjectMonitor {

		private boolean m_deleted;
		private Vector3D m_lastPosition;
		private boolean m_needsUpdate;
		private SpatialLocation m_spatialLoc;
		private final String m_spatialLocID;

		private float m_width;
		private String m_monitoredID;
		private String m_monitoredSA;

		public VisualObjectMonitor(SceneManager _sm, WorkingMemoryChange _wmc)
				throws SubarchitectureProcessException {

			m_monitoredID = _wmc.m_address.m_id;
			m_monitoredSA = _wmc.m_address.m_subarchitecture;

			// filter for changes to the object
			setupFilters(m_monitoredID, m_monitoredSA);

			// create an id to always write to
			m_spatialLocID = newDataID();

			m_needsUpdate = false;

			// woo... let's do something
			added();

		}

		public String getSpatialObjectID() {
			return m_spatialLocID;
		}

		public boolean isDeleted() {
			return m_deleted;
		}

		private void added() {
			try {
				log("VisualObjectMonitor.added() " + m_spatialLocID);
				assert (!existsOnWorkingMemory(m_spatialLocID));

				// get most recent copy of vo and convert
				convertToSpatialLocation();
				m_lastPosition = sceneObject().m_bbox.m_centroid;
				m_width = sceneObject().m_bbox.m_size.m_x;
				addToWorkingMemory(m_spatialLocID, m_spatialLoc);
				// and lock it so that only this comp can alter it
				lockEntry(m_spatialLocID, WorkingMemoryPermissions.LOCKED_OD);
				m_deleted = false;

				// store for manager
				m_locationMonitors.put(m_spatialLocID, m_spatialLoc);
			}
			catch (SubarchitectureProcessException e) {
				println(e);
			}

		}

		private void convertToSpatialLocation()
				throws SubarchitectureProcessException {
			m_spatialLoc = sceneObjectToSpatialLocation(m_monitoredID,
					m_monitoredSA, sceneObject());
		}

		private void deleted() {
			try {
				log("VisualObjectMonitor.deleted() " + m_spatialLocID);
				deleteFromWorkingMemory(m_spatialLocID);
				m_deleted = true;
				m_locationMonitors.remove(m_spatialLocID);
				// implicit unlock in delete
			}
			catch (SubarchitectureProcessException e) {
				println(e);
			}

		}

		private double distance(Vector3D _a, Vector3D _b) {
			return Math.sqrt(Math.pow((_a.m_x - _b.m_x), 2)
					+ Math.pow((_a.m_y - _b.m_y), 2));

		}

		private void overwritten() {
			try {
				log("VisualObjectMonitor.overwritten() " + m_spatialLocID);

				if (significantChange()) {
					m_needsUpdate = true;
				}
				else {
					log("VisualObjectMonitor.overwritten() ignoring change "
							+ m_spatialLocID);
				}
			}
			catch (SubarchitectureProcessException e) {
				println(e);
			}

		}
		private SceneObject sceneObject()
				throws SubarchitectureProcessException {
			// TODO recheck cache
			return (SceneObject) getWorkingMemoryEntry(m_monitoredID,
					m_monitoredSA).getData();
		}

		private void setupFilters(String monitoredID, String monitoredSA)
				throws SubarchitectureProcessException {
			// filter for this object being added (in case it is ever removed)
			addChangeFilter(ChangeFilterFactory.createChangeFilter(
					SceneObject.class, WorkingMemoryOperation.ADD, "",
					monitoredID, monitoredSA, FilterRestriction.ALL_SA),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							added();
						}
					});

			// filter for this object being added (in case it is ever removed)
			addChangeFilter(ChangeFilterFactory.createChangeFilter(
					SceneObject.class, WorkingMemoryOperation.OVERWRITE, "",
					monitoredID, monitoredSA, FilterRestriction.ALL_SA),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							overwritten();
						}
					});

			// filter for this object being added (in case it is ever removed)
			addChangeFilter(ChangeFilterFactory.createChangeFilter(
					SceneObject.class, WorkingMemoryOperation.DELETE, "",
					monitoredID, monitoredSA, FilterRestriction.ALL_SA),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							deleted();
						}
					});
		}

		/**
		 * If the object has moved by more than it's bbox width
		 * 
		 * @return
		 * @throws SubarchitectureProcessException
		 */
		private boolean significantChange()
				throws SubarchitectureProcessException {
			return distance(m_lastPosition, sceneObject().m_bbox.m_centroid) > m_width;
		}

		private void update() throws SubarchitectureProcessException {
			if (m_needsUpdate && !m_deleted) {
				log("VisualObjectMonitor.update():  doing update "
						+ m_spatialLocID);
				convertToSpatialLocation();
				overwriteWorkingMemory(m_spatialLocID, m_spatialLoc);
				m_locationMonitors.put(m_spatialLocID, m_spatialLoc);
			}
		}
	}

	private static final double SCENE_DIFF_MILLIS = 10000;

	/**
	 * This is too near to static objects
	 */
	private static final double TOO_NEAR = 0.03;

	private Camera m_camera;

	private BALTTime m_lastUpdate;

	private String m_visionSA;

	private final Hashtable<String, VisualObjectMonitor> m_voMonitors;
	private final Hashtable<String, SpatialLocation> m_locationMonitors;

	int increment = 0;

	/**
	 * Contains a list of static points in the world that we should also
	 * consider in scences
	 */
	private final ArrayList<String> m_staticLocations;

	/**
	 * @param _id
	 */
	public SceneManager(String _id) {
		super(_id);
		m_camera = null;
		m_voMonitors = new Hashtable<String, VisualObjectMonitor>();
		m_locationMonitors = new Hashtable<String, SpatialLocation>();
		m_staticLocations = new ArrayList<String>();
	}

	@Override
	protected void redrawGraphics3D() {
		// super.redrawGraphics3D();

		if (m_currentSceneID != null) {
			// log("drawing: " + m_currentSceneID);
			try {
				SpatialScene scene = getScene(m_currentSceneID);
				for (String locID : scene.m_locations) {
					SpatialLocation loc = getSpatialLocation(locID);
					// drawPoint3D(loc.m_centroid.m_x, loc.m_centroid.m_y,
					// loc.m_centroid.m_z, 200, 0, 0, FAT.value);
					drawBox3D(loc.m_centroid.m_x, loc.m_centroid.m_y,
							loc.m_centroid.m_z, 0.02, 0.02, 0.02, 0, 150, 0,
							FAT.value);
				}

			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
			}

		}
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

			// catch new objects for processing
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					SceneObject.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							sceneObjectAdded(_wmc);
						}
					});

			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					SceneChanged.class, WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								queueSceneUpdate(_wmc.m_address);
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
							}
						}
					});

			// for the time being just take camera additions
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
					Camera.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							changedCameraPose(_wmc);
						}
					});

			// change filter for state additions to the scene
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					AddToScene.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								addToScene(_wmc.m_address.m_id);
							}
							catch (AlreadyExistsOnWMException e) {
								e.printStackTrace();
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
							}
						}
					});

			// // for the time being just take camera additions
			// addChangeFilter(VisionOntology.CAMERA_TYPE,
			// WorkingMemoryOperation.OVERWRITE, false,
			// new WorkingMemoryChangeReceiver() {
			//
			// public void workingMemoryChanged(
			// WorkingMemoryChange _wmc) {
			// changedCameraPose(_wmc);
			// }
			// });

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.components.InspectableComponent#redrawGraphics2D()
	 */
	@Override
	protected void redrawGraphics2D() {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _goalID) {

		try {
			updateSpatialScene();
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _goalID) {
		log("SceneManager.taskRejected()");
	}

	/**
	 * Add static points to the scene.
	 * 
	 * @param _id
	 * @throws SubarchitectureProcessException
	 * @throws AlreadyExistsOnWMException
	 */
	private void addToScene(String _id) throws AlreadyExistsOnWMException,
			SubarchitectureProcessException {
		AddToScene additions = (AddToScene) getWorkingMemoryEntry(_id)
				.getData();

		for (SpatialLocation loc : additions.m_locations) {
			String locID = newDataID();
			addToWorkingMemory(locID, loc, OperationMode.BLOCKING);
			m_staticLocations.add(locID);
			log("new static objects added: " + locID);
		}

		// now we need to update
		proposeSceneUpdate();
	}

	/**
	 * @throws SubarchitectureProcessException
	 */
	private void changedCameraPose(WorkingMemoryChange _wmc) {
		try {
			if (m_visionSA == null) {
				m_visionSA = _wmc.m_address.m_subarchitecture;
			}

			// get the camera pose from wherever it is
			CASTTypedData<?> camData = getWorkingMemoryEntry(_wmc.m_address);

			Camera camera = (Camera) camData.getData();

			if (camera.m_num == 0) {
				log("got left camera");

				if (m_camera != null) {
					// check to see what the movement is
					// significant

					// not happening for us

					// if (significantChange(camera, m_camera))
					// {
					//
					// // HACK Find a nicer way of doing this!
					// queueSceneUpdate(_wmc[i].m_address,
					// WorkingMemoryOperation.GET);
					// }
				}
				// first call just sets the camera
				else {
					m_camera = camera;

				}
			}
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Creates a new monitoring object for the visual object
	 * 
	 * @param _address
	 * @throws SubarchitectureProcessException
	 */
	private void monitor(WorkingMemoryChange _address)
			throws SubarchitectureProcessException {

		m_voMonitors.put(_address.m_address.m_id, new VisualObjectMonitor(this,
				_address));
	}

	// /*
	// * (non-Javadoc)
	// *
	// * @see
	// cast.architecture.subarchitecture.PrivilegedManagedProcess#runComponent()
	// */
	// @Override
	// protected void runComponent() {
	// while (true) {
	// redrawGraphicsNow();
	// sleepProcess(100);
	// }
	// }

	/**
	 * 
	 */
	private void proposeSceneUpdate() {
		String taskID = newTaskID();

		try {
			proposeInformationProcessingTask(taskID, SpatialGoals.UPDATE_SCENE);
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @param _address
	 * @throws SubarchitectureProcessException
	 */
	private void queueSceneUpdate(WorkingMemoryAddress _address)
			throws SubarchitectureProcessException {

		log("SceneManager.queueSceneUpdate()");

		SceneChanged sc = (SceneChanged) getWorkingMemoryEntry(_address)
				.getData();

		// now work on the processed flag!
		if (updateAllowed(sc)) {
			proposeSceneUpdate();
		}
	}

	private SpatialLocation sceneObjectToSpatialLocation(String _voID,
			String _voSA, SceneObject _sceneObject) {
		SpatialLocation spatialObject = new SpatialLocation(
				new WorkingMemoryPointer(CASTUtils.typeName(SceneObject.class),
						new WorkingMemoryAddress(_voID, _voSA)),
				_sceneObject.m_bbox.m_centroid);
		return spatialObject;
	}

	/**
	 * @param _sc
	 * @return
	 */
	private boolean updateAllowed(SceneChanged _sc) {

		// only act if scene has been processed... waits for segmentor
		if (_sc.m_sceneProcessed) {

			// the first time
			if (m_lastUpdate == null) {
				// store time of update
				m_lastUpdate = ProcessLauncher.getBALTTime();
				return true;
			}

			// get current time
			BALTTime currentTime = ProcessLauncher.getBALTTime();
			// if the difference is great enough
			if (NativeProcessLauncher.toMillis(NativeProcessLauncher.timeDiff(
					m_lastUpdate, currentTime)) > SCENE_DIFF_MILLIS) {
				m_lastUpdate = ProcessLauncher.getBALTTime();
				return true;
			}
			else {
				println("rejecting quick update within " + SCENE_DIFF_MILLIS + "ms");
			}
		}

		return false;

	}

	/**
	 * When the visual scene is static create a new "spatial scene" to trigger
	 * relationship processing. This currently ignores any context and starts
	 * each scene from scratch. This involves getting all visible objects from
	 * vision.
	 * 
	 * @throws SubarchitectureProcessException
	 */
	private void updateSpatialScene() throws SubarchitectureProcessException {
		log("SceneManager.updateSpatialScene()");

		if (m_camera == null) {
			throw new SpatialSubarchitectureException("Camera object not set");
		}

		// start with the static locations
		ArrayList<String> spatialLocationIDs = new ArrayList<String>();

		// cause object monitors to rewrite necessary
		for (VisualObjectMonitor vom : m_voMonitors.values()) {
			if (!vom.isDeleted()) {
				try {
					vom.update();
					// add object locations
					spatialLocationIDs.add(vom.getSpatialObjectID());
				}
				catch (SubarchitectureProcessException e) {
					println(e);
				}
			}
		}

		// then add static locs if they're not too close to objects
		// TODO fix lazy coding
		ArrayList<String> remove = new ArrayList<String>(m_staticLocations
				.size());

		for (String staticID : m_staticLocations) {
			if (!nearLocations(staticID, spatialLocationIDs)) {
				log("including static loc: " + staticID);
				spatialLocationIDs.add(staticID);
			}
			else {
				log("not including static loc: " + staticID);
				remove.add(staticID);
				deleteFromWorkingMemory(staticID);
			}
		}

		// remove any static locs we don't want
		m_staticLocations.removeAll(remove);

		// create a scene!
		SpatialScene scene = new SpatialScene(m_camera.m_pose,
				NativeProcessLauncher.getBALTTime(),
				new String[spatialLocationIDs.size()],
				new SpatialRelationship[0], new SpatialRelationship[0],
				new SpatialRelationship[0], new SpatialRelationship[0],
				new SpatialRelationship[0], new String[0]);

		// write in ids
		for (int i = 0; i < spatialLocationIDs.size(); i++) {
			scene.m_locations[i] = spatialLocationIDs.get(i);
		}

		// write to wm
		m_currentSceneID = newDataID();
		addToWorkingMemory(m_currentSceneID, scene);

		println("scene added to working memory containing "
				+ spatialLocationIDs.size() + " objects");
	}

	@Override
	protected SpatialLocation getSpatialLocation(String _spatialObjectAddress)
			throws SubarchitectureProcessException {
		SpatialLocation loc = m_locationMonitors.get(_spatialObjectAddress);
		if (loc == null) {
			return super.getSpatialLocation(_spatialObjectAddress);
		}
		else {
			return loc;
		}
	}

	private boolean nearLocations(String _locID,
			ArrayList<String> _locationIDList)
			throws SubarchitectureProcessException {
		SpatialLocation inputLoc = getSpatialLocation(_locID);
		for (String locID : _locationIDList) {
			SpatialLocation listLoc = getSpatialLocation(locID);
			if (near(listLoc, inputLoc)) {
				return true;
			}
		}
		return false;
	}

	private boolean near(SpatialLocation _statLoc,
			SpatialLocation _spatialObject) {
		//
		log(SpatialUtils.toString(_statLoc.m_centroid)
				+ " "
				+ SpatialUtils.toString(_spatialObject.m_centroid)
				+ " ->"
				+ SpatialUtils.distance(_statLoc.m_centroid,
						_spatialObject.m_centroid) + " < " + TOO_NEAR);

		return SpatialUtils.distance(_statLoc.m_centroid,
				_spatialObject.m_centroid) < TOO_NEAR;
	}

	boolean isMonitored(String _id) {
		return m_voMonitors.containsKey(_id);
	}

	/**
	 * When a new scene object is added that we've never seen before, this
	 * creates a new monitor for it.
	 * 
	 * @param _wmc
	 */
	void sceneObjectAdded(WorkingMemoryChange _wmc) {
		try {
			if (!isMonitored(_wmc.m_address.m_id)) {
				monitor(_wmc);
			}
		}
		catch (SubarchitectureProcessException e) {
			println(e);
		}
	}

}
