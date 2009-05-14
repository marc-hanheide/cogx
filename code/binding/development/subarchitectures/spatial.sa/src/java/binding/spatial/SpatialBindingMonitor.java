/**
 * 
 */
package binding.spatial;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.Map;
import java.util.Properties;

import org.cognitivesystems.common.autogen.Math.Vector3DWithConfidence;

import planning.autogen.Action;
import planning.autogen.PlanningStatus;
import spatial.autogen.AddToScene;
import spatial.autogen.RepositionLocation;
import spatial.autogen.SpatialLocation;
import spatial.autogen.SpatialRelationship;
import spatial.autogen.SpatialRelationshipTargetQuery;
import spatial.autogen.SpatialScene;
import spatial.util.SpatialSubarchitectureException;
import spatial.util.SpatialUtils;
import BindingFeatures.DebugString;
import BindingFeatures.Location;
import BindingFeatures.RelationLabel;
import BindingFeaturesCommon.TemporalFrameType;
import BindingFeaturesCommon.TruthValue;
import BindingQueries.MakeProxyUnavailable;
import balt.core.data.Pair;
import binding.abstr.AbstractMonitor;
import binding.common.BindingComponentException;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.core.data.CASTDataCache;

/**
 * Binding proxy monitor for the spatial subarchitecture. The monitor copies
 * spatial locations and objects to the binding wm. It endeavours to make as few
 * updates to binding structures as possible.
 * 
 * 
 * On a scene change the following things can happen: - objects appear,
 * disappear or move - relationships appear, disaopear or change value
 * 
 * @author nah
 */
public class SpatialBindingMonitor extends AbstractMonitor {

	protected HashMap<String, String> m_locationProxyMappings;

	private WorkingMemoryAddress m_lastSceneAddr;

	private final CASTDataCache<SpatialScene> m_sceneCache;

	private final CASTDataCache<SpatialLocation> m_locationCache;

	private final Hashtable<String, LocationMonitor> m_locationMonitors;

	private final HashMap<String, String> m_targetProxyMappings;

	private final Hashtable<String, HashSet<BindingRelation>> m_relations;

	private final Hashtable<String, Pair<String, RepositionLocation>> m_repoMap;

	private class LocationMonitor implements WorkingMemoryChangeReceiver {

		private boolean m_overwritten = false;

		public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			assert (_wmc.m_operation == WorkingMemoryOperation.OVERWRITE);
			m_overwritten = true;
		}

		public void reset() {
			m_overwritten = false;
		}

		public boolean hasBeenOverwritten() {
			return m_overwritten;
		}

	}

	/**
	 * @param _id
	 */
	public SpatialBindingMonitor(String _id) {
		super(_id);

		m_locationProxyMappings = new HashMap<String, String>();
		m_locationMonitors = new Hashtable<String, LocationMonitor>();
		m_relations = new Hashtable<String, HashSet<BindingRelation>>();

		m_sceneCache = new CASTDataCache<SpatialScene>(this, SpatialScene.class);
		m_locationCache = new CASTDataCache<SpatialLocation>(this,
				SpatialLocation.class);

		m_targetProxyMappings = new HashMap<String, String>();

		m_repoMap = new Hashtable<String, Pair<String, RepositionLocation>>();

		m_completeActions = new HashMap<String, TriBool>();

		m_processed = new HashSet<String>();
	}

	private final SpatialScene scene(String _id)
			throws SubarchitectureProcessException {
		return m_sceneCache.getData(_id);
	}

	private final SpatialLocation location(String _id)
			throws SubarchitectureProcessException {
		return m_locationCache.getData(_id);
	}

	@Override
	protected void makeProxyUnavailable(MakeProxyUnavailable _makeProxyUnavailable) throws DoesNotExistOnWMException, SubarchitectureProcessException {
		log("NOT making proxy unavailable: " + _makeProxyUnavailable.m_proxyID);
	}
	
	
	
	/**
	 * creates the Binding Proxy and stores it. Returns the WM address.
	 */
	private void processScene(WorkingMemoryAddress _wma)
			throws SubarchitectureProcessException {

		if (haveProcessed(_wma.m_id)) {
			return;
		}

		// get the new scene data
		SpatialScene newScene = scene(_wma.m_id);

		if (newScene.m_activeComponents.length == 0) {

			log("SpatialBindingMonitor.handleNewScene()");

			// delete all old relations: there are so many permutations on how
			// they
			// can change that it is too hard to maintain
			for (String proxyAddr : m_locationProxyMappings.values()) {
				deleteProxyRelations(proxyAddr);
			}

			// store current scene address, and register a change filter so we
			// know
			// what happens to it
			m_lastSceneAddr = _wma;
			log("new scene at: " + CASTUtils.toString(m_lastSceneAddr));

			// create a list of all existing locations so we can check if any
			// have
			// been removed
			HashSet<String> currentLocations = new HashSet<String>(
					m_locationProxyMappings.keySet());

			// go through the list of locations in the current scene
			String locID = null;
			for (int i = 0; i < newScene.m_locations.length; i++) {

				locID = newScene.m_locations[i];

				// ok now we've seen this
				currentLocations.remove(locID);

				// if it's a new location
				if (!hasProxy(locID)) {
					// then create a new proxy
					newLocationProxy(locID);
				}
				// if it's moved since the last scene change
				else if (hasChangedPosition(locID)) {
					// then update it's location
					updateLocationProxy(locID);
				}

			}

			// now remove any remaining proxies that don't feature in the
			// current
			// scene
			for (String id : currentLocations) {
				removeLocationProxy(id);
			}

			// reset monitors to wait for changes to location positions
			for (LocationMonitor locMon : m_locationMonitors.values()) {
				locMon.reset();
			}

			// and then add relations
			handleSceneRelationships(newScene);

			bindNewProxies();

			// now we've got a new scene, complete any actions that
			// should've
			// resulted in a new scene
			signalActions();

			processed(_wma.m_id);

		}
		else {
			log("not binding while components still active: "
					+ Arrays.toString(newScene.m_activeComponents));
		}
	}

	/**
	 * Scenes we've process
	 */
	private final HashSet<String> m_processed;

	private void processed(String _id) {
		m_processed.add(_id);

	}

	private boolean haveProcessed(String _id) {
		return m_processed.contains(_id);

	}

	private void updateLocationProxy(String _locID)
			throws BindingComponentException, SubarchitectureProcessException {
		String proxyAddr = m_locationProxyMappings.get(_locID);
		assert (proxyAddr != null);

		HashSet<String> deleteThese = new HashSet<String>(1);
		deleteThese.add(CASTUtils.typeName(Location.class));

		// load and delete old
		changeExistingProxy(proxyAddr, deleteThese);

		// add the location
		Location loc = addLocationToCurrentProxy(_locID);
		storeCurrentProxy();

		log("updating location at: " + loc.m_location.m_x + " "
				+ loc.m_location.m_y + " " + +loc.m_location.m_z);

	}

	private Location addLocationToCurrentProxy(String _locID)
			throws SubarchitectureProcessException, BindingComponentException {
		Location loc = new Location();
		loc.m_location = location(_locID).m_centroid;
		addFeatureToCurrentProxy(loc);
		return loc;
	}

	private void removeLocationProxy(String _locID)
			throws SubarchitectureProcessException {

		assert (m_locationProxyMappings.containsKey(_locID));
		assert (m_locationMonitors.containsKey(_locID));

		log("SpatialBindingMonitor.removeLocationProxy(): " + _locID);

		// delete location proxy
		deleteExistingProxy(m_locationProxyMappings.get(_locID));
//		 delete any relations it featured in
		deleteProxyRelations(m_locationProxyMappings.get(_locID));
		m_locationProxyMappings.remove(_locID);

		// need to check, because it could've been updated previously
		if (m_targetProxyMappings.containsKey(_locID)) {
			// and it's desired counterpart
			deleteExistingProxy(m_targetProxyMappings.get(_locID));
			m_targetProxyMappings.remove(_locID);

		}
		

		removeChangeFilter(m_locationMonitors.remove(_locID));
		// and delete associated relations
	}

	private boolean hasChangedPosition(String _locID) {
		assert (m_locationMonitors.containsKey(_locID));
		return m_locationMonitors.get(_locID).hasBeenOverwritten();
	}

	private boolean hasProxy(String _locID) {
		return m_locationProxyMappings.containsKey(_locID);
	}

	private void newLocationProxy(String _locID)
			throws BindingComponentException, SubarchitectureProcessException {

		// register a receiver for overwrites to the location, i.e. changes in
		// postition
		LocationMonitor locMon = new LocationMonitor();
		addChangeFilter(ChangeFilterFactory.createIDFilter(_locID,
				WorkingMemoryOperation.OVERWRITE), locMon);
		m_locationMonitors.put(_locID, locMon);

		startNewBasicProxy();
		Location loc = addLocationToCurrentProxy(_locID);
		changeTemporalFrameOfCurrentProxy(TemporalFrameType.PERCEIVED);
		String locationProxyAddr = storeCurrentProxy();
		m_locationProxyMappings.put(_locID, locationProxyAddr);
		log("adding location at: " + loc.m_location.m_x + " "
				+ loc.m_location.m_y + " " + +loc.m_location.m_z);

		// also add a dummy way point that can be used as a target position
		// should this need to be moved
		startNewBasicProxy();
		loc = addLocationToCurrentProxy(_locID);
		changeTemporalFrameOfCurrentProxy(TemporalFrameType.DESIRED);
		locationProxyAddr = storeCurrentProxy();
		m_targetProxyMappings.put(_locID, locationProxyAddr);

	}

	/**
	 * For each change received, add the relations specified by that
	 * 
	 * 
	 * @param _wmc
	 */
	private boolean handleSceneRelationships(SpatialScene scene) {

		log("SpatialBindingMonitor.handleNewRelationships()");

		try {
			// get the scene

			if (scene.m_activeComponents.length == 0) {

				SpatialRelationship[] rels = null;
				String label = null;

				// HACK This is the quick way, but ugly
				rels = scene.m_leftRelationships;
				label = SpatialUtils.LEFT_OF;
				for (SpatialRelationship rel : rels) {
					addProxyRelationship(rel.m_landmark, rel.m_target, label);
				}

				rels = scene.m_rightRelationships;
				label = SpatialUtils.RIGHT_OF;
				for (SpatialRelationship rel : rels) {
					addProxyRelationship(rel.m_landmark, rel.m_target, label);
				}

				rels = scene.m_frontRelationships;
				label = SpatialUtils.FRONT_OF;
				for (SpatialRelationship rel : rels) {
					addProxyRelationship(rel.m_landmark, rel.m_target, label);
				}

				rels = scene.m_backRelationships;
				label = SpatialUtils.BACK_OF;
				for (SpatialRelationship rel : rels) {
					addProxyRelationship(rel.m_landmark, rel.m_target, label);
				}

				rels = scene.m_proximityRelationships;
				label = SpatialUtils.NEAR;
				for (SpatialRelationship rel : rels) {
					addProxyRelationship(rel.m_landmark, rel.m_target, label);
				}

				bindNewProxies();

				// now we've got a new scene, complete any actions that
				// should've
				// resulted in a new scene
				signalActions();
				return true;
			}
			else {
				log("not adding rels while components still active: "
						+ Arrays.toString(scene.m_activeComponents));
			};

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
		return false;

	}

	/**
	 * @param _landmark
	 * @param _target
	 * @param _label
	 * @throws SubarchitectureProcessException
	 * @throws BindingComponentException
	 */
	private void addProxyRelationship(String _landmark, String _target,
			String _label) throws BindingComponentException,
			SubarchitectureProcessException {

		// get ids for proxies
		String landmarkProxyAddr = m_locationProxyMappings.get(_landmark);
		if (landmarkProxyAddr == null) {
			throw new SpatialSubarchitectureException(
					"no binding proxy for spatial object:" + _landmark);
		}

		String targetProxyAddr = m_locationProxyMappings.get(_target);
		if (targetProxyAddr == null) {
			throw new SpatialSubarchitectureException(
					"no binding proxy for spatial object:" + _target);
		}

		// relations are now all wiped, so this can't happen
		assert (!relationExists(targetProxyAddr, landmarkProxyAddr, _label));
		// if (!relationExists(targetProxyAddr, landmarkProxyAddr, _label)) {
		// println("WARNING: not adding relations to binding wm");
		log("adding relationship: " + landmarkProxyAddr + " " + targetProxyAddr
				+ " " + _label);

		// // add relation to binding wm
		// = addSimpleRelation(targetProxyAddr, landmarkProxyAddr,
		// _label, TemporalFrameType.PERCEIVED);
		//		

		// Now, try to add relation as a proxy too:
		// HACK code stolen from simple relation, but needed to add extra
		// features
		startNewRelationProxy();
		RelationLabel label = new RelationLabel();
		label.m_label = _label;
		addFeatureToCurrentProxy(label);
		DebugString info = new DebugString();
		info.m_debugString = "from: " + targetProxyAddr + " to: "
				+ landmarkProxyAddr;
		addFeatureToCurrentProxy(info);
		addOutPortToCurrentProxy(landmarkProxyAddr, "to");
		addOutPortToCurrentProxy(targetProxyAddr, "from");
		changeTemporalFrameOfCurrentProxy(TemporalFrameType.PERCEIVED);
		addOtherSourceIDToCurrentProxy(m_sourceID, TruthValue.NEGATIVE);
		String relAddr = storeCurrentProxy();

		// store locally for later comparisons
		storeRelation(targetProxyAddr, landmarkProxyAddr, _label, relAddr);
		// }
		// else {
		// log("leaving existing relationship: " + landmarkProxyAddr + " "
		// + targetProxyAddr + " " + _label);
		//
		// }

	}
	// /**
	// * @throws SubarchitectureProcessException
	// */
	// private void newScene() throws SubarchitectureProcessException {
	//
	// log("SpatialBindingMonitor.newScene()");
	//
	// // reset monitoring of unbound proxies
	// if (m_unboundProxyAddresses.size() != 0) {
	// // cancel any open proxies
	//
	// boolean done = false;
	//
	// for (String proxyAddr : m_unboundProxyAddresses) {
	//
	// done = false;
	//
	// // again the hacky way around read/write speed
	// // difference
	// while (!done) {
	// try {
	// // set current proxy to unbound one
	// changeExistingProxy(proxyAddr);
	// done = true;
	// }
	// catch (SubarchitectureProcessException e) {
	// log("retrying changeExistingProxyID for: " + proxyAddr);
	// sleepProcess(10);
	// }
	// }
	//
	// log("cancelling unbound proxy");
	// // then get rid of it
	// cancelCurrentProxy();
	// }
	// m_unboundProxyAddresses.clear();
	// }
	//
	// // stop waiting for things
	// if (m_unmatchedProxies.size() != 0) {
	// log("removing proxy filter");
	// removeFilterForProxies();
	// m_unmatchedProxies.clear();
	// }
	//
	// // delete current proxies
	// for (String proxyAddr : m_currentProxies) {
	// deleteExistingProxy(proxyAddr);
	// }
	//
	// m_currentProxies.clear();
	//
	// log("last scene address: " + m_lastSceneAddr);
	//
	// if (m_lastSceneAddr != null) {
	// removeChangeFilterForScene(m_lastSceneAddr);
	// m_lastSceneAddr = null;
	// }
	//
	// }

	// /**
	// * @param _wma
	// * @throws SubarchitectureProcessException
	// */
	// private void removeChangeFilterForScene(WorkingMemoryAddress _wma)
	// throws SubarchitectureProcessException {
	// removeChangeFilter(SpatialOntology.SPATIAL_SCENE_TYPE,
	// WorkingMemoryOperation.OVERWRITE, "", _wma.m_id,
	// _wma.m_subarchitecture, true);
	// }

	// /**
	// * @param _wmc
	// */
	// private void handleBindingProxy(WorkingMemoryChange _wmc) {
	// // determines whether a particular binding proxy has a
	// // matching source data feature we're waiting for
	//
	// // log("SpatialBindingMonitor.newBindingProxy()");
	//
	// // this should be checked somewhere else!
	// if (_wmc.m_src.equals(getProcessIdentifier())) {
	// log("ignoring own proxies: " + _wmc.m_src + " - "
	// + _wmc.m_address.m_id);
	//
	// return;
	// }
	//
	// try {
	//
	// // get the binding proxy
	// BindingProxy bindingProxy =
	// getBindingProxy(_wmc.m_address.m_id);
	//
	// String featureAddress =
	// findFeature(bindingProxy,
	// BindingOntology.SOURCE_DATA_TYPE);
	//
	// if (featureAddress != null) {
	// // log("has source data");
	// SourceData sd =
	// getFeature(featureAddress, SourceData.class);
	//
	// if (sd.m_type.equals(VisionOntology.SCENE_OBJECT_TYPE)) {
	//
	// // now check if we have a proxy wait for this
	// String sceneObjectID = sd.m_address.m_id;
	//
	// if (m_unmatchedProxies.containsKey(sceneObjectID)) {
	//
	// log("found a match for a waiting ibc");
	//
	// // get proxy and remove from map
	// String unmatchedProxyID =
	// m_unmatchedProxies.get(sceneObjectID);
	//
	// // might run in to problems with read/write
	// // timings
	// boolean done = false;
	// while (!done) {
	// try {
	// changeExistingProxy(unmatchedProxyID);
	// done = true;
	// }
	// catch (SubarchitectureProcessException e) {
	// log("retrying wm query");
	// }
	// sleepProcess(10);
	// }
	//
	// // update retrieved proxy with new
	// // feature
	// addExistingProxyIDFeature(_wmc.m_address);
	//
	// // store proxy to wm
	// String matchedProxyAddr = storeCurrentProxy();
	// m_currentProxies.add(matchedProxyAddr);
	//
	// m_unmatchedProxies.remove(sceneObjectID);
	//
	// // remove filter if no more unmatched ibcs
	// if (m_unmatchedProxies.size() == 0) {
	// log("binding matched proxies");
	// removeFilterForProxies();
	// bindNewProxies();
	// }
	//
	// }
	// }
	//
	// }
	//
	// }
	// catch (SubarchitectureProcessException e) {
	// e.printStackTrace();
	// }
	//
	// }

	// simple struct storing the rels written to wm
	private static class BindingRelation {

		/**
		 * Target proxy id
		 */
		public String m_target;

		/**
		 * Landmark proxy id
		 */
		public String m_landmark;

		/**
		 * Rel label
		 */
		public String m_label;

		/**
		 * Rel proxy id
		 */
		public String m_addr;

		private int m_hash;

		public BindingRelation(String _target, String _landmark, String _label,
				String _addr) {
			m_target = _target;
			m_landmark = _landmark;
			m_label = _label;
			m_addr = _addr;

			// create hash
			StringBuffer sb = new StringBuffer(m_target);
			sb.append(m_landmark);
			sb.append(m_label);
			sb.append(m_addr);
			m_hash = sb.toString().hashCode();
		}

		@Override
		public boolean equals(Object _obj) {
			if (_obj instanceof BindingRelation) {
				BindingRelation rel = (BindingRelation) _obj;
				return rel.m_target.equals(m_target)
						&& rel.m_landmark.equals(m_landmark)
						&& rel.m_label.equals(m_label)
						&& rel.m_addr.equals(m_addr);
			}
			else {
				return false;
			}
		}

		@Override
		public int hashCode() {
			return m_hash;
		}

		public boolean matches(String _targetProxyAddr,
				String _landmarkProxyAddr, String _label) {
			return _targetProxyAddr.equals(m_target)
					&& _landmarkProxyAddr.equals(m_landmark)
					&& _label.equals(m_label);
		}

		@Override
		public String toString() {
			StringBuffer sb = new StringBuffer();
			sb.append("[REL ");
			sb.append(m_target);
			sb.append(" ");
			sb.append(m_label);
			sb.append(" ");
			sb.append(m_landmark);
			sb.append(" @ ");
			sb.append(m_addr);
			sb.append("]");

			return sb.toString();
		}
	}

	/**
	 * Keep a track of all binding relations created to avoid duplication and to
	 * allow clean-up
	 * 
	 */
	private void storeRelation(String _targetProxyAddr,
			String _landmarkProxyAddr, String _relLabel, String _relAddr) {

		BindingRelation rel = new BindingRelation(_targetProxyAddr,
				_landmarkProxyAddr, _relLabel, _relAddr);

		// store relations in map
		storeRelation(_targetProxyAddr, rel);
		storeRelation(_landmarkProxyAddr, rel);

		log("storing: " + rel);

	}

	private void storeRelation(String _proxyAddr, BindingRelation _rel) {
		if (!m_relations.containsKey(_proxyAddr)) {
			m_relations.put(_proxyAddr, new HashSet<BindingRelation>());
		}
		assert (!m_relations.get(_proxyAddr).contains(_rel));
		m_relations.get(_proxyAddr).add(_rel);
	}

	/**
	 * Remove all relations this proxy was involved in
	 * 
	 * @param _proxyID
	 * @throws SubarchitectureProcessException
	 */
	private void deleteProxyRelations(String _proxyID)
			throws SubarchitectureProcessException {
		assert(_proxyID != null);
		HashSet<BindingRelation> rels = m_relations.remove(_proxyID);
		// could be legitimately null if location exists with no rels
		if (rels != null) {
			for (BindingRelation rel : rels) {
				log("deleting: " + rel);
				// delete from wm
				deleteExistingProxy(rel.m_addr);
				// delete from storage
				deleteStoredProxyRelation(rel);
			}
		}
	}

	private void deleteStoredProxyRelation(BindingRelation _rel) {
		for (HashSet<BindingRelation> rels : m_relations.values()) {
			rels.remove(_rel);
		}
	}

	/**
	 * 
	 * Determines whether this relation was previously added to wm.
	 * 
	 * @param _targetProxyAddr
	 * @param _landmarkProxyAddr
	 * @param _label
	 * @return
	 */
	private boolean relationExists(String _targetProxyAddr,
			String _landmarkProxyAddr, String _label) {
		// only need to check in one direction
		HashSet<BindingRelation> rels = m_relations.get(_targetProxyAddr);
		if (rels != null) {
			for (BindingRelation rel : rels) {
				if (rel.matches(_targetProxyAddr, _landmarkProxyAddr, _label)) {
					return true;
				}
			}
		}
		return false;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see binding.abstr.AbstractMonitor#configure(java.util.Properties)
	 */
	@Override
	public void configure(Properties _config) {
		super.configure(_config);
		m_sourceID = m_subarchitectureID;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		super.start();

		// System.out.println("\033[35mtesting");

		try {

			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					SpatialScene.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {

						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								processScene(_wmc.m_address);
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
							}
						}
					});

			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					SpatialScene.class, WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								processScene(_wmc.m_address);
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
							}
						}
					});

			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					Action.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							repositonLocation(_wmc.m_address);

						}

					});

			// // change filter for adding new positions based on desired
			// // relations
			// addChangeFilter(BindingOntology.BINDING_PROXY_TYPE,
			// WorkingMemoryOperation.ADD, false,
			// new WorkingMemoryChangeReceiver() {
			//
			// public void workingMemoryChanged(
			// WorkingMemoryChange _wmc) {
			// try {
			//
			// // ignore own changes
			// if (_wmc.m_src.equals(getProcessIdentifier())) {
			// return;
			// }
			//
			// inspectRelation(_wmc);
			//
			// }
			// catch (SubarchitectureProcessException e) {
			// e.printStackTrace();
			// }
			// }
			//
			// });

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	private void repositonLocation(WorkingMemoryAddress _address) {
		try {

			Action action = (Action) getWorkingMemoryEntry(_address).getData();

			if (!action.m_action.m_type.equals(CASTUtils
					.typeName(RepositionLocation.class))) {
				log("Can't handle action: " + action.m_action.m_type);
				return;
			}

			RepositionLocation repoRequest = (RepositionLocation) getWorkingMemoryEntry(
					action.m_action.m_address).getData();

			log("SpatialBindingMonitor.repositonLocation(): target="
					+ repoRequest.m_targetID + " landmark="
					+ repoRequest.m_landmarkID + " rel="
					+ SpatialUtils.enum2string(repoRequest.m_rel));

			// check that we have a target proxy for the request target
			if (!m_targetProxyMappings.containsValue(repoRequest.m_targetID)) {
				println("No target proxy: " + repoRequest.m_targetID);
				return;
			}

			// check that we have a target proxy for the request target
			if (!m_locationProxyMappings
					.containsValue(repoRequest.m_landmarkID)) {
				println("No landmark proxy: " + repoRequest.m_landmarkID);
				return;
			}

			String landmarkLocationID = "";

			// find the id of the object the proxy was generated from
			for (String locID : m_locationProxyMappings.keySet()) {
				if (m_locationProxyMappings.get(locID).equals(
						repoRequest.m_landmarkID)) {
					landmarkLocationID = locID;
					break;
				}
			}

			// shouldn't be possible!
			assert (landmarkLocationID != "");

			log("SpatialBindingMonitor.repositonLocation(): location="
					+ landmarkLocationID + " rel="
					+ SpatialUtils.enum2string(repoRequest.m_rel));

			// now see if we can get a point for the new target position
			SpatialRelationshipTargetQuery query = new SpatialRelationshipTargetQuery(
					repoRequest.m_rel, landmarkLocationID, 1, m_lastSceneAddr,
					new Vector3DWithConfidence[0]);

			String queryID = newDataID();

			m_repoMap.put(queryID, new Pair<String, RepositionLocation>(
					_address.m_id, repoRequest));

			// listen for query answer
			addChangeFilter(ChangeFilterFactory.createIDFilter(queryID,
					WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							try {
								queryAnswered(_wmc.m_address.m_id);
								// remove after update
								removeChangeFilter(this);
							}
							catch (SubarchitectureProcessException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}

					});
			addToWorkingMemory(queryID, query);

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	private final <K, V> K findKey(String _value, Map<K, V> _map) {
		for (K key : _map.keySet()) {
			if (_map.get(key).equals(_value)) {
				return key;
			}
		}
		return null;
	}

	private void actionComplete(String _actionID, TriBool _success) {
		m_completeActions.put(_actionID, _success);
	}

	private void signalActions() throws SubarchitectureProcessException {
		for (String actionID : m_completeActions.keySet()) {
			Action action = (Action) getWorkingMemoryEntry(actionID).getData();
			action.m_status = PlanningStatus.COMPLETE;
			action.m_succeeded = m_completeActions.get(actionID);
			overwriteWorkingMemory(actionID, action);
		}
		m_completeActions.clear();
	}

	private void queryAnswered(String _queryID)
			throws DoesNotExistOnWMException, SubarchitectureProcessException {
		// it really should
		assert (m_repoMap.containsKey(_queryID));
		Pair<String, RepositionLocation> repoPair = m_repoMap.remove(_queryID);
		RepositionLocation repo = repoPair.m_second;
		String actionID = repoPair.m_first;

		// get the answered query
		SpatialRelationshipTargetQuery query = (SpatialRelationshipTargetQuery) getWorkingMemoryEntry(
				_queryID).getData();

		if (query.m_targets.length == 0) {
			println("Unable to reposition: " + repo.m_targetID);
			actionComplete(actionID, TriBool.triFalse);
		}
		else {
			// create a new location to exist here
			SpatialLocation loc = new SpatialLocation(
					new WorkingMemoryPointer(CASTUtils
							.typeName(RepositionLocation.class),
							new WorkingMemoryAddress(actionID,
									getSubarchitectureID())),
					query.m_targets[0].m_vector);

			// delete old desired proxy
			String locationID = findKey(repo.m_targetID, m_targetProxyMappings);
			assert (locationID != null);
			deleteExistingProxy(repo.m_targetID);
			m_targetProxyMappings.remove(locationID);

			// and add loc to scene
			log("adding requested location at: " + loc.m_centroid.m_x + " "
					+ loc.m_centroid.m_y + " " + +loc.m_centroid.m_z);
			AddToScene additions = new AddToScene(new SpatialLocation[]{loc});
			addToWorkingMemory(newDataID(), additions);

			actionComplete(actionID, TriBool.triTrue);
		}

		// clean up
		deleteFromWorkingMemory(_queryID);
	}

	private final HashMap<String, TriBool> m_completeActions;

	@Override
	protected void runComponent() {
//		sleepProcess(20000);
//		println("DOING THE DUMMY THING");
//
//		String targetID = m_targetProxyMappings.values().iterator().next();
//		String landmarkID = m_locationProxyMappings.values().iterator().next();
//
//		RepositionLocation repo = new RepositionLocation(landmarkID, targetID,
//				SpatialRelationshipType.SPATIAL_LEFT);
//
//		String repoID = newDataID();
//		
//		Action act = new Action(new WorkingMemoryPointer(CASTUtils.typeName(RepositionLocation.class), new WorkingMemoryAddress(repoID, getSubarchitectureID())), PlanningStatus.INCOMPLETE, TriBool.triIndeterminate);
//		
//		try {
//			addToWorkingMemory(repoID, repo);
//			addToWorkingMemory(newDataID(), act);
//		}
//		catch (AlreadyExistsOnWMException e) {
//			e.printStackTrace();
//		}
//		catch (SubarchitectureProcessException e) {
//			e.printStackTrace();
//		}

	}

}
