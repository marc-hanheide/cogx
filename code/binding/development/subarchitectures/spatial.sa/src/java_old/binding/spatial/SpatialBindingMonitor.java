/**
 * 
 */
package binding.spatial;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.Properties;

import spatial.autogen.SpatialLocation;
import spatial.autogen.SpatialRelationship;
import spatial.autogen.SpatialScene;
import spatial.ontology.SpatialOntologyFactory;
import spatial.util.SpatialSubarchitectureException;
import spatial.util.SpatialUtils;
import BindingData.BindingProxy;
import BindingData.BindingProxyType;
import BindingFeatures.Location;
import BindingFeatures.RelationLabel;
import BindingFeatures.TemporalFrame;
import BindingFeaturesCommon.TemporalFrameType;
import binding.abstr.AbstractMonitor;
import binding.common.BindingComponentException;
import binding.util.BindingUtils;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import cast.core.data.CASTDataCache;
import cast.core.data.CachedCASTData;

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

	private Hashtable<String, LocationMonitor> m_locationMonitors;

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
	}

	private final SpatialScene scene(String _id)
			throws SubarchitectureProcessException {
		return m_sceneCache.getData(_id);
	}

	private final SpatialLocation location(String _id)
			throws SubarchitectureProcessException {
		return m_locationCache.getData(_id);
	}

	private final ArrayList<CachedCASTData<SpatialLocation>> locations(
			SpatialScene _scene) {

		ArrayList<CachedCASTData<SpatialLocation>> locations = new ArrayList<CachedCASTData<SpatialLocation>>(
				_scene.m_locations.length);

		for (String id : _scene.m_locations) {
			locations.add(m_locationCache.get(id));
		}

		return locations;

	}

	/**
	 * creates the Binding Proxy and stores it. Returns the WM address.
	 */
	private void handleNewScene(WorkingMemoryAddress _wma)
			throws SubarchitectureProcessException {

		log("SpatialBindingMonitor.handleNewScene()");

		// delete all old relations: there are so many permutations on how they
		// can change that it is too hard to maintain
		for (String proxyAddr : m_locationProxyMappings.values()) {
			deleteProxyRelations(proxyAddr);
		}

		// store current scene address, and register a change filter so we know
		// what happens to it
		m_lastSceneAddr = _wma;
		log("new scene at: " + CASTUtils.toString(m_lastSceneAddr));
		addChangeFilterForScene(m_lastSceneAddr);

		// get the new scene data
		SpatialScene newScene = scene(_wma.m_id);

		// create a list of all existing locations so we can check if any have
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

		// now remove any remaining proxies that don't feature in the current
		// scene
		for (String id : currentLocations) {
			removeLocationProxy(id);
		}

		// reset monitors to wait for changes to location positions
		for (LocationMonitor locMon : m_locationMonitors.values()) {
			locMon.reset();
		}

		bindNewProxies();

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

		// delete any relations it featured in
		deleteProxyRelations(m_locationProxyMappings.get(_locID));

		// clean up associated mappings and filters
		m_locationProxyMappings.remove(_locID);
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
		addPerceivedTemporalFrame();
		String locationProxyAddr = storeCurrentProxy();
		m_locationProxyMappings.put(_locID, locationProxyAddr);
		log("adding location at: " + loc.m_location.m_x + " "
				+ loc.m_location.m_y + " " + +loc.m_location.m_z);
	}

	private void addPerceivedTemporalFrame() throws BindingComponentException,
			SubarchitectureProcessException {
		TemporalFrame tf = new TemporalFrame();
		tf.m_temporalFrame = TemporalFrameType.PERCEIVED;
		addFeatureToCurrentProxy(tf);
	}

	/**
	 * @param _wma
	 * @throws SubarchitectureProcessException
	 */
	private void addChangeFilterForScene(WorkingMemoryAddress _wma)
			throws SubarchitectureProcessException {

		addChangeFilter(ChangeFilterFactory.createChangeFilter(
				SpatialScene.class, WorkingMemoryOperation.OVERWRITE, "",
				_wma.m_id, _wma.m_subarchitecture, FilterRestriction.LOCAL_SA),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleNewRelationships(_wmc);
					}
				});
	}

	/**
	 * For each change received, add the relations specified by that
	 * 
	 * 
	 * @param _wmc
	 */
	private void handleNewRelationships(WorkingMemoryChange _wmc) {

		log("SpatialBindingMonitor.handleNewRelationships()");

		try {

			// get the scene
			SpatialScene scene = scene(_wmc.m_address.m_id);

			SpatialRelationship[] rels = null;
			String label = null;

			// HACK This is the quick way, but ugly
			if (_wmc.m_src.contains("left")) {
				rels = scene.m_leftRelationships;
				label = SpatialUtils.LEFT_OF;
			}
			else if (_wmc.m_src.contains("right")) {
				rels = scene.m_rightRelationships;
				label = SpatialUtils.RIGHT_OF;
			}
			else if (_wmc.m_src.contains("front")) {
				rels = scene.m_frontRelationships;
				label = SpatialUtils.FRONT_OF;
			}
			else if (_wmc.m_src.contains("back")) {
				rels = scene.m_backRelationships;
				label = SpatialUtils.BACK_OF;
			}
			else if (_wmc.m_src.contains("prox")) {
				rels = scene.m_proximityRelationships;
				label = SpatialUtils.NEAR;
			}
			else {
				throw new SpatialSubarchitectureException(
						"Unknown source component: " + _wmc.m_src);
			}

			for (SpatialRelationship rel : rels) {
				addProxyRelationship(rel.m_landmark, rel.m_target, label);
			}

			bindNewProxies();

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}

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

		// add relation to binding wm
		String relAddr = addSimpleRelation(targetProxyAddr, landmarkProxyAddr,
				_label, TemporalFrameType.PERCEIVED);

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

	private final Hashtable<String, HashSet<BindingRelation>> m_relations;

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

								handleNewScene(_wmc.m_address);
							}
							catch (SubarchitectureProcessException e) {
								e.printStackTrace();
							}
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

}
