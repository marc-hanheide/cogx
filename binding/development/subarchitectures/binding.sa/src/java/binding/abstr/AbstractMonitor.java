/**
 * 
 */
package binding.abstr;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Properties;
import java.util.TreeSet;

import BindingData.BINDING_SUBARCH_NOCHECK_CONFIG_KEY;
import BindingData.BestUnionsForProxy;
import BindingData.BindTheseProxies;
import BindingData.BindingProxy;
import BindingData.BindingProxyDeletionTask;
import BindingData.BindingProxyState;
import BindingData.BindingProxyType;
import BindingData.BindingScore;
import BindingData.ExplicitFeatureDeletionTask;
import BindingData.FeaturePointer;
import BindingData.NonMatchingUnions;
import BindingData.PortType;
import BindingData.ProxyPort;
import BindingData.ProxyPorts;
import BindingData.bindTheseProxiesMonitorTokenID;
import BindingData.binderTokenID;
import BindingFeatures.CreationTime;
import BindingFeatures.DebugString;
import BindingFeatures.Group;
import BindingFeatures.OtherSourceID;
import BindingFeatures.RelationLabel;
import BindingFeatures.Salience;
import BindingFeatures.Singular;
import BindingFeatures.SourceID;
import BindingFeatures.TemporalFrame;
import BindingFeatures.ThisProxyID;
import BindingFeaturesCommon.EndTime;
import BindingFeaturesCommon.ParentFeature;
import BindingFeaturesCommon.StartTime;
import BindingFeaturesCommon.TemporalFrameType;
import BindingFeaturesCommon.TruthValue;
import BindingQueries.MakeProxyUnavailable;
import balt.corba.autogen.FrameworkBasics.BALTTime;
import balt.jni.NativeProcessLauncher;
import binding.FeaturePointerSet;
import binding.common.BindingComponentException;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
import cast.core.WMAComparator;
import cast.core.data.CASTData;
/**
 * @author henrikj
 */
public abstract class AbstractMonitor extends AbstractBindingReader {
    
    // / features from an updated proxy must be deleted to avoid
    // / memory leaks
    private FeaturePointerSet m_featuresToDelete;

    /**
     * defines the ID of the monitored SA
     */
    protected String m_sourceID;

    /**
     * The ID of the latest added sourceID, must be added to OtherSorceID so
     * that the scorer can avoid scoring it with a SourceID of the same proxy
     */
    protected String m_sourceIDAddress;

    /**
     * if currently built proxy is a new one, this is false
     */
    protected Boolean m_updatingProxy;
    /**
     * the ID of the build proxy.
     */
    protected String m_currentProxyID;
    /**
     * The proxy which is currently being built up by features being added to it
     * (and WM)
     */
    protected BindingProxy m_currentlyBuiltProxy;

    /*
     * A list of proxies added to the WM but which have not yet been bound
     * (bindNewProxies binds the proxies and empties the list)
     */
    protected HashSet<String> m_unboundProxyAddresses;


    /**
     * A list of subarchs not checked for union ids before updating
     */
    protected final HashSet<String> m_uncheckedSAs;

    
    // ! true iff memfun start() called
    private boolean m_startCalled;

    // ! true iff binderReadySignal has been called
    private boolean m_binderReady;

    /*
     * Maps from proxy IDs to the address of the inportlist
     */
    protected HashMap<String, String> m_proxyID2inportsID;

    // ! proxies that belong to this monitor
    private HashSet<String> m_ownedProxyIDs;

    private final TreeSet<WorkingMemoryAddress> m_tokens;

    /*
     * 
     */
    protected TemporalFrameType m_temporalFrameOfCurrentProxy;

    protected AbstractMonitor(String _id) {
	super(_id);
	m_unboundProxyAddresses = new HashSet<String>();
	m_ownedProxyIDs = new HashSet<String>();
	m_featuresToDelete = new FeaturePointerSet();
	m_currentProxyID = "";
	m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	m_sourceIDAddress = null;
	m_proxyID2inportsID = new HashMap();
	m_temporalFrameOfCurrentProxy = TemporalFrameType.NA;
	m_startCalled = false;
	// m_hasUpdateToken = false;
	m_tokens = new TreeSet<WorkingMemoryAddress>(new WMAComparator());
	m_uncheckedSAs = new HashSet<String>(1);
    }

    /**
     * Deletes the features from WM. Only to be used when cancelling an unstored
     * proxy, therefore private
     * 
     * @throws SubarchitectureProcessException
     */
    private void _deleteFeatures(FeaturePointerSet _features)
	throws SubarchitectureProcessException {
	for (FeaturePointer pointer : _features) {
	    // deleteFromWorkingMemory(pointer.m_address);
	    log("Deleting feature task added for feature: " + pointer.m_address);
	    ExplicitFeatureDeletionTask del_task = new ExplicitFeatureDeletionTask();
	    del_task.m_featureID = pointer.m_address;
	    addToWorkingMemory(newDataID(), getBindingSA(), del_task,
			       OperationMode.BLOCKING);
	}
    }

    /**
     * factory for ParentFeatures
     */
    protected ParentFeature defaultParentFeature() {
	return new ParentFeature(TruthValue.POSITIVE, m_currentProxyID);
    }

    /**
     * not so elegant... but what do you do when there Corba insists on a Java
     * array to extend?
     */
    private FeaturePointer[] addFeatureToArray(FeaturePointer[] _features,
					       FeaturePointer _feature) {

	FeaturePointer[] newFeats = new FeaturePointer[_features.length + 1];
	System.arraycopy(_features, 0, newFeats, 0, _features.length);
	newFeats[_features.length] = _feature;
	_features = newFeats;
	return _features;
    }

    protected String featureToCASTFeatureTypeString(Object _obj) {
	return CASTUtils.typeName(_obj);
    }
    /*
     * protected String featureToCASTFeatureTypeString(Object _obj) throws
     * BindingComponentException {
     * 
     * String ret = ((BindingOntology) BindingOntologyFactory.getOntology())
     * .className2FeatureName().get( _obj.getClass().getName());
     * 
     * if (ret == null) { throw new BindingComponentException( "Not a feature
     * according to " + this.getClass().getName() + " : " +
     * _obj.getClass().getName()); }
     * 
     * return ret; }
     */
    private <FeatureT> FeaturePointer storeFeature(FeatureT _anyFeature,
						   TruthValue _truthValue) throws SubarchitectureProcessException {
	FeaturePointer feature = new FeaturePointer();

	try {
	    feature.m_address = newDataID();
	    feature.m_type = CASTUtils.typeName(_anyFeature);
	    // featureToCASTFeatureTypeString(_anyFeature);
	    feature.m_immediateProxyID = m_currentProxyID;

	    setParent(_anyFeature, new ParentFeature(_truthValue,
						     m_currentProxyID));

	    // log("storing feature: " + feature.m_address + " " +
	    // feature.m_type);
	    addToWorkingMemory(feature.m_address, getBindingSA(),
			       // feature.m_type,
			       _anyFeature, OperationMode.BLOCKING);
	}
	catch (BindingComponentException e) {
	    println(e.getLocalizedMessage());
	    e.printStackTrace();
	}
	return feature;
    }

    private <FeatureT> void setParent(FeatureT _anyFeature,
				      ParentFeature _parent) throws BindingComponentException {
	try {
	    Field parentField = _anyFeature.getClass().getField("m_parent");
	    parentField.set(_anyFeature, _parent);
	}
	catch (Exception e) {
	    throw new BindingComponentException(
						"Error setting m_parent field reflectively for class: "
						+ _anyFeature.getClass(), e);
	}
    }

    /**
     * Adds the creation time to the proxy
     */
    protected FeaturePointer addCreationTimeToCurrentProxy()
	throws BindingComponentException, SubarchitectureProcessException {
	CreationTime creationTime = new CreationTime();
	creationTime.m_creationTime = NativeProcessLauncher.getBALTTime();
	FeaturePointer ret = addFeatureToCurrentProxy(creationTime,
						      TruthValue.POSITIVE);
	return ret;
    }

    /**
     * Stores a feature to WM and adds the adress to the set of features
     * pointers of the proxy
     */
    protected FeaturePointer addFeatureToCurrentProxy(Object _feature,
						      TruthValue _truthValue) throws BindingComponentException,
										     SubarchitectureProcessException {
	if (m_currentlyBuiltProxy == null) {
	    throw (new BindingComponentException(
						 "Attempt to add feature to m_proxyFeatures before initiating it with startNewProxy"));
	}
	FeaturePointer ptr = storeFeature(_feature, _truthValue);
	m_currentlyBuiltProxy.m_proxyFeatures = addFeatureToArray(
								  m_currentlyBuiltProxy.m_proxyFeatures, ptr);
	return ptr;
    }

    /**
     * Stores a feature to WM and adds the adress to the set of features
     * pointers of the proxy
     */
    protected FeaturePointer addFeatureToCurrentProxy(Object _feature)
	throws BindingComponentException, SubarchitectureProcessException {
	return addFeatureToCurrentProxy(_feature, TruthValue.POSITIVE);
    }

    // private stuff below this point...

    /**
     * Adds a relation from proxy on WM address ID _from to proxy on WM address
     * ID _to with _label to denote the type of relation. Returns the stored
     * relation proxy's ID.
     */
    protected String addSimpleRelation(String _from, String _to, String _label,
				       TemporalFrameType _frame) throws BindingComponentException,
									SubarchitectureProcessException {
	if (m_currentlyBuiltProxy != null) {
	    throw (new BindingComponentException(
						 "Cannot add relation while working on a proxy since the relation is itself a proxy"));
	}
	// SimpleRelation rel = new SimpleRelation();
	// /*
	// * This test is reasonable, but it doesn't work since it may
	// be
	// * performed before all writes are made...
	// *
	// if(!getWorkingMemoryEntry(_from).getType().equals(BindingOntology.BINDING_PROXY_TYPE))
	// {
	// * throw(new BindingComponentException("_from not a Binding
	// * proxy")); }
	// *
	// if(!getWorkingMemoryEntry(_to).getType().equals(BindingOntology.BINDING_PROXY_TYPE))
	// {
	// * throw(new BindingComponentException("_to not a Binding
	// * proxy")); }
	// */
	// // //log("gonna makerelation from " + _from + " to " + _to +
	// "
	// // ("
	// // + _label + ")");
	// rel.m_from = _from;
	// rel.m_to = _to;
	// rel.m_label = _label;
	//
	// // OBSOLETE SOON:
	// addToWorkingMemory(newDataID(), m_bindingSA,
	// BindingOntology.SIMPLE_RELATION_TYPE, rel);

	// Now, try to add relation as a proxy too:
	startNewRelationProxy();

	// as it's acting as a simple rel, make sure it doesn't bind
	// within subarch
	// addOtherSourceIDToCurrentProxy(m_sourceID, true);

	RelationLabel label = new RelationLabel();
	label.m_label = _label;
	addFeatureToCurrentProxy(label);
	DebugString info = new DebugString();
	info.m_debugString = "from: " + _from + " to: " + _to;
	addFeatureToCurrentProxy(info);

	addOutPortToCurrentProxy(_to, "to");
	addOutPortToCurrentProxy(_from, "from");
	changeTemporalFrameOfCurrentProxy(_frame);
	// save the relation
	String relationID = storeCurrentProxy();
	return relationID;
    }

    protected void addOutPortToCurrentProxy(String _proxyID, String _portLabel)
	throws BindingComponentException {
	if (m_currentlyBuiltProxy.m_type != BindingProxyType.RELATION) {
	    throw (new BindingComponentException(
						 "only add outports for relation proxies, please."));
	}
	ProxyPort port = new ProxyPort();
	port.m_type = PortType.PROXY; // for now... ok...
	port.m_label = _portLabel;
	port.m_proxyID = _proxyID;
	port.m_ownerProxyID = m_currentProxyID;
	ProxyPort[] new_outports = new ProxyPort[m_currentlyBuiltProxy.m_outPorts.m_ports.length + 1];
	System.arraycopy(m_currentlyBuiltProxy.m_outPorts.m_ports, 0,
			 new_outports, 0,
			 m_currentlyBuiltProxy.m_outPorts.m_ports.length);

	new_outports[m_currentlyBuiltProxy.m_outPorts.m_ports.length] = port;
	m_currentlyBuiltProxy.m_outPorts.m_ports = new_outports;
    }

    protected void updateInports(ProxyPorts _ports)// _proxyID, String
    // _relationID, PortType
    // _portType, String
    // _portLabel)
	throws BindingComponentException, SubarchitectureProcessException {
	return;
	// deprecated	for (int i = 0; i < _ports.m_ports.length; ++i) {
	// deprecated	    
	// deprecated	    String proxyID = _ports.m_ports[i].m_proxyID;
	// deprecated	    String inportsID = m_proxyID2inportsID.get(proxyID);
	// deprecated	    try {
	// deprecated		if (inportsID == null) {
	// deprecated		    CASTData from_proxy_data = getWorkingMemoryEntry(proxyID,
	// deprecated								     getBindingSA());
	// deprecated		    BindingProxy from_proxy = (BindingProxy) from_proxy_data
	// deprecated			.getData();
	// deprecated		    inportsID = from_proxy.m_inPortsID;
	// deprecated		    m_proxyID2inportsID.put(proxyID, inportsID);
	// deprecated		}
	// deprecated		
	// deprecated		CASTData inportsData = getWorkingMemoryEntry(inportsID,
	// deprecated							     getBindingSA());
	// deprecated		ProxyPorts inports = (ProxyPorts) inportsData.getData();
	// deprecated		
	// deprecated		ProxyPort[] new_inports = new ProxyPort[inports.m_ports.length + 1];
	// deprecated		System.arraycopy(inports.m_ports, 0, new_inports, 0,
	// deprecated				 inports.m_ports.length);
	// deprecated		new_inports[inports.m_ports.length] = 
	// deprecated		    new ProxyPort(
	// deprecated				  _ports.m_ports[i].m_type, _ports.m_ports[i].m_label,
	// deprecated				  _ports.m_ports[i].m_ownerProxyID,
	// deprecated				  _ports.m_ports[i].m_proxyID);
	// deprecated		
	// deprecated		// assert(haveLatestVersion(inportsID, getBindingSA()));
	// deprecated		overwriteWorkingMemory(inportsID, 
	// deprecated				       getBindingSA(),
	// deprecated				       new ProxyPorts(inports.m_ownerProxyID, new_inports),
	// deprecated				       OperationMode.BLOCKING);
	// deprecated		
	// deprecated		m_unboundProxyAddresses.add(proxyID);
	// deprecated	    }
	// deprecated	    catch (DoesNotExistOnWMException _e) {
	// deprecated		log("could not update inports of " + proxyID + " : "
	// deprecated		    + _e.toString());
	// deprecated	    }
	// deprecated	}
    }

    /**
     * adds the source ID to the proxy. Make sure your instantiated monitor sets
     * m_sourceID first.
     */
    protected FeaturePointer addSourceIDToCurrentProxy()
	throws BindingComponentException, SubarchitectureProcessException {
	SourceID sourceID = new SourceID();
	if (m_sourceID == null) {
	    throw new BindingComponentException(
						"AbstractMonitor.m_sourceID not initialized. This should be done in the implemented monitor.");

	}
	sourceID.m_sourceID = m_sourceID;
	sourceID.m_monitorID = getProcessIdentifier();
	FeaturePointer ret = addFeatureToCurrentProxy(sourceID);
	m_sourceIDAddress = ret.m_address;
	return ret;
    }

    /**
     * specifies that the proy should be scored base on unions stemming from a
     * particular subarch (or vice versa, if negated)
     */
    protected FeaturePointer addOtherSourceIDToCurrentProxy(String _id,
							    TruthValue _truthValue) throws BindingComponentException,
											   SubarchitectureProcessException {
	OtherSourceID otherSourceID = new OtherSourceID();

	otherSourceID.m_otherSourceID = _id;
	otherSourceID.m_parent = defaultParentFeature();
	// otherSourceID.m_thisSourceIDRef = m_sourceIDAddress;
	FeaturePointer ret = addFeatureToCurrentProxy(otherSourceID,
						      _truthValue);
	return ret;
    }

    /**
     * After all new proxies are added and relations between them defined it is
     * time to bind them. This stores a BindTheseProxies object onto the WM and
     * resets m_unboundProxyAdresses to an empty list
     */
    protected void bindNewProxies() throws SubarchitectureProcessException {
	if (0 != m_unboundProxyAddresses.size()) {

	    if (!hasBindTheseProxiesMonitorToken()) {
		acquireBindTheseProxiesMonitorToken();
	    }
	    if (!hasBinderToken()) {
		acquireBinderToken();
	    }
	    releaseBinderToken();

	    try {
		BindTheseProxies bindThese = new BindTheseProxies();
		bindThese.m_proxyIDs = new String[m_unboundProxyAddresses
						  .size()];

		// can't use an index in a hashset :(
		int i = 0;
		for (String unboundProxyAddress : m_unboundProxyAddresses) {
		    log("bindNewProxies:" + unboundProxyAddress);
		    // m_ownedProxyIDs.add(bindThese.m_proxyIDs[i]);
		    // nah: above line is bad bad bad
		    m_ownedProxyIDs.add(unboundProxyAddress);
		    bindThese.m_proxyIDs[i++] = unboundProxyAddress;
		}

		for (String pid : m_ownedProxyIDs) {
		    log("owned: " + pid);
		}

		addToWorkingMemory(newDataID(), getBindingSA(), bindThese,
				   OperationMode.BLOCKING);
		m_unboundProxyAddresses.clear();

		// now delete any features that have been replaced/removed
		_deleteFeatures(m_featuresToDelete);
		m_featuresToDelete.clear();
	    }
	    catch (SubarchitectureProcessException _e) {
		releaseBindTheseProxiesMonitorToken();
		// if (m_hasUpdateToken)
		// releaseUpdateProxyMonitorToken();
		throw _e;
	    }
	    releaseBindTheseProxiesMonitorToken();
	    // if (m_hasUpdateToken)
	    // releaseUpdateProxyMonitorToken();
	}
    }

    /**
     * Cancels the creation of a proxy, deletes all created features up to this
     * point and makes it possible to start a new proxy.
     * 
     * @throws SubarchitectureProcessException
     */
    protected void cancelCurrentProxy() throws SubarchitectureProcessException {

	if (m_currentlyBuiltProxy == null) {
	    return;
	}

	FeaturePointerSet deleteThese = new FeaturePointerSet();

	for (FeaturePointer pointer : m_currentlyBuiltProxy.m_proxyFeatures) {
	    deleteThese.add(pointer);
	}

	_deleteFeatures(deleteThese);

	m_currentlyBuiltProxy = null;
	m_sourceIDAddress = null;
    }

    /**
     * calls protected void changeExistingProxy(_proxyAddr, new HashSet<String>())
     * 
     * @param _proxyAddr
     * @throws SubarchitectureProcessException
     */
    protected void changeExistingProxy(String _proxyAddr)
	throws SubarchitectureProcessException, BindingComponentException {
	// log("in changeExistingProxy(String _proxyAddr)");
	changeExistingProxy(_proxyAddr, new HashSet<String>());

	/*
	 * if (m_currentlyBuiltProxy != null) { throw new
	 * BindingComponentException( "An existing proxy can not be loaded until
	 * the previous one is stored or cancelled"); } m_currentProxyID =
	 * _proxyAddr; m_updatingProxy = true; // get proxy to be changed
	 * BindingProxy proxy = null; proxy = getBindingProxy(_proxyAddr); if
	 * (proxy != null) { // create new binding proxy locally
	 * m_currentlyBuiltProxy = deepCopy(proxy); for (FeaturePointer pointer :
	 * m_currentlyBuiltProxy.m_proxyFeatures) { // println("changed proxy
	 * has feature: " + // pointer.m_type);
	 * 
	 * if (pointer.m_type .equals(BindingOntology.SOURCE_ID_TYPE))
	 * m_sourceIDAddress = pointer.m_address; } } else { // reset default
	 * address m_currentProxyID = ""; m_updatingProxy = false;
	 * 
	 * throw new BindingComponentException( "at changing proxy in abstract
	 * monitor: missing proxy at address: " + _proxyAddr); }
	 */
    }

    private <T> void disallow(Class<T> _class,
			      HashSet<String> _deleteTheseFeatures)
	throws BindingComponentException {
	if (_deleteTheseFeatures.contains(CASTUtils.typeName(_class))) {
	    throw new RuntimeException(
				       CASTUtils.typeName(_class)
				       + " must not be in _deleteTheseFeatures in changeExistingProxy");
	}
    }

    
    private final boolean isCheckedSA(String _sa) {
    	return !m_uncheckedSAs.contains(_sa);
    }
    
    /**
     * Loads an existing proxy and marks all its features for deletion.
     * 
     * @param _proxyAddr
     * @param _deleteTheseTypes,
     *            features of these types will be deleted from the existing
     *            binding proxy, for example, if you want to change the colour
     *            of an object, do not keep it, and then add new colour.
     *            Possibly we need something more sophisticated here when
     *            updating features with more than one value.
     * @throws SubarchitectureProcessException
     */
    protected void changeExistingProxy(String _proxyAddr,
				       HashSet<String> _deleteTheseFeatures)
	throws SubarchitectureProcessException, BindingComponentException {
	// log("in AbstractMonitor::changeExistingProxy(const
	// string& _proxyAddr, const set<string>&
	// _deleteTheseFeatureTypes)");

	if (m_currentlyBuiltProxy != null) {
	    throw new BindingComponentException(
						"An existing proxy can not be loaded until the previous one is stored or cancelled");
	}

	// if (!m_hasUpdateToken)
	// acquireUpdateProxyMonitorToken();

	if (!hasBindTheseProxiesMonitorToken()) {
	    acquireBindTheseProxiesMonitorToken();
	}
	if (!hasBinderToken()) {
	    acquireBinderToken();
	}

	disallow(CreationTime.class, _deleteTheseFeatures);
	disallow(ThisProxyID.class, _deleteTheseFeatures);
	disallow(Group.class, _deleteTheseFeatures);
	disallow(Singular.class, _deleteTheseFeatures);

	m_currentProxyID = _proxyAddr;
	m_updatingProxy = true;

	BindingProxy proxy = getBindingProxy(_proxyAddr);

	if (isCheckedSA(getBindingSA())
			&& proxy.m_unionID.equals("") && // i.e. unbound proxy
	    !m_unboundProxyAddresses.contains(m_currentProxyID)) {// i.e.,
	    // it's
	    // been
	    // added
	    // to
	    // bindtheseproxies,
	    // or it
	    // been
	    // cancelled
	    throw new RuntimeException(
				       "ILLEGAL!!! Attempt to update a proxy while binder is working on it (or one that has been cancelled). ProxyID: "
				       + m_currentProxyID);
	}
	

	if (proxy != null) {

	    m_currentlyBuiltProxy = deepCopy(proxy);

	    m_currentlyBuiltProxy.m_proxyFeatures = new FeaturePointer[0];
	    // then, store the existing features for deletion and copy
	    // features
	    // we want to delete
	    m_currentlyBuiltProxy.m_updates = proxy.m_updates + 1;
	    for (FeaturePointer pointer : proxy.m_proxyFeatures) {
		// if (pointer.m_type.equals(BindingOntology.SOURCE_ID_TYPE))
		if (pointer.m_type.equals(CASTUtils.typeName(SourceID.class)))
		    m_sourceIDAddress = pointer.m_address;

		if (_deleteTheseFeatures.contains(pointer.m_type)) {
		    // log("will not copy this feature when updating
		    // proxy: "
		    // + pointer.m_type);
		    m_featuresToDelete.add(pointer);
		}
		else { // if not deleted, then we need to add it to the new
		    // proxy
		    log("will copy this feature when updating proxy: "
			+ pointer.m_type);
		    m_currentlyBuiltProxy.m_proxyFeatures = addFeatureToArray(
									      m_currentlyBuiltProxy.m_proxyFeatures, pointer);
		}
	    }
	    log("Size of the feature array: ["
		+ m_currentlyBuiltProxy.m_proxyFeatures.length
		+ "] (original: [" + proxy.m_proxyFeatures.length
		+ "]) for proxy type [" + m_currentlyBuiltProxy.m_type
		+ "]");
	    if (m_currentlyBuiltProxy.m_type.value() == BindingProxyType._GROUP) {
		boolean has_group_feature = false;
		for (int i = 0; i < m_currentlyBuiltProxy.m_proxyFeatures.length; i++) {
		    log("Scanning feature: type ["
			+ m_currentlyBuiltProxy.m_proxyFeatures[i].m_type
			+ "]");
		    if (m_currentlyBuiltProxy.m_proxyFeatures[i].m_type
			.equals(CASTUtils.typeName(Group.class))) {
			has_group_feature = true;
		    }
		}
		if (!has_group_feature) {
		    throw new RuntimeException(
					       "After explicitly copying nonexcluded features: Group feature not in group proxy: "
					       + m_currentProxyID);
		}
	    }

	}
	else {
	    // reset default address
	    m_currentProxyID = "";
	    m_updatingProxy = false;
	    throw new RuntimeException(
				       "at changing proxy in abstract monitor: missing proxy at address: "
				       + _proxyAddr);
	}

	// log("out AbstractMonitor::changeExistingProxy(String
	// _proxyAddr, HashMap<String> _deleteTheseFeatureTypes");
    }

    /**
     * Simply deletes the proxy and all its features
     * 
     * @param _proxyAddr
     * @throws SubarchitectureProcessException
     */
    protected void deleteExistingProxy(String _proxyAddr)
	throws SubarchitectureProcessException {

	log("deleteExistingProxy: " + _proxyAddr);
	if (m_currentProxyID.equals(_proxyAddr)) {
	    cancelCurrentProxy();
	    return;
	}

	assert (m_ownedProxyIDs.contains(_proxyAddr));

	if (!hasBindTheseProxiesMonitorToken()) {
	    acquireBindTheseProxiesMonitorToken();
	}
	if (!hasBinderToken()) {
	    acquireBinderToken();
	}

	// releaseBinderToken(); // give it back to binder right away
	try {
	    BindingProxy proxy = getBindingProxy(_proxyAddr);

	    log("Proxy deletion task added for: " + _proxyAddr);
	    BindingProxyDeletionTask del_task = new BindingProxyDeletionTask();
	    del_task.m_proxyID = _proxyAddr;

	    // just in case:
	    m_unboundProxyAddresses.remove(_proxyAddr);

	    addToWorkingMemory(newDataID(), getBindingSA(), del_task,
			       OperationMode.BLOCKING);

	    m_ownedProxyIDs.remove(_proxyAddr);

	    /*
	     * FeaturePointerSet deleteThese = new FeaturePointerSet(); for
	     * (FeaturePointer pointer : proxy.m_proxyFeatures) {
	     * deleteThese.add(pointer); }
	     */

	    // 1st, delete proxy
	    // deleteFromWorkingMemory(_proxyAddr);
	    // 2nd, delete features, not any more... let someone else do it
	    // properly
	    // _deleteFeatures(deleteThese);
	}
	catch (DoesNotExistOnWMException _e) {
	    releaseBindTheseProxiesMonitorToken();
	    releaseBinderToken();
	    throw _e;
	}
	releaseBindTheseProxiesMonitorToken();
	releaseBinderToken();
    }

    /**
     * @param _ibcAddress
     * @return
     * @throws SubarchitectureProcessException
     */
    protected BindingProxy getBindingProxy(String _proxyAddr)
	throws SubarchitectureProcessException {

	BindingProxy ibc = null;

	try {
	    CASTData<?> wme = getWorkingMemoryEntry(_proxyAddr, getBindingSA());
	    ibc = (BindingProxy) wme.getData();
	}
	catch (SubarchitectureProcessException e) {
	    log("No BindingProxy at given address: " + _proxyAddr);
	}
	return ibc;

    }

    /**
     * starts a new proxy (OBS, it overwrites information if you havent stored
     * the last proxy)
     * 
     * @throws BindingComponentException
     */
    protected void startNewBasicProxy() throws BindingComponentException,
					       SubarchitectureProcessException {
	_startNewProxy(BindingProxyType.BASIC);
	m_temporalFrameOfCurrentProxy = TemporalFrameType.NA;
    }

    /**
     * starts a new proxy (OBS, it overwrites information if you havent stored
     * the last proxy)
     * 
     * @throws BindingComponentException
     */
    protected void startNewRelationProxy() throws BindingComponentException,
						  SubarchitectureProcessException {
	_startNewProxy(BindingProxyType.RELATION);
	m_temporalFrameOfCurrentProxy = TemporalFrameType.PERCEIVED;
	if (m_currentlyBuiltProxy == null) {
	    throw new RuntimeException("m_currentlyBuiltProxy == null!!!");
	}

    }

    protected void startNewGroupProxy(short _groupSize)
	throws BindingComponentException, SubarchitectureProcessException {
	startNewBasicProxy();
	makeCurrentProxyAGroup(_groupSize);
	m_temporalFrameOfCurrentProxy = TemporalFrameType.NA;
	if (m_currentlyBuiltProxy == null) {
	    throw new RuntimeException("m_currentlyBuiltProxy == null!!!");
	}
    }

    protected void makeCurrentProxyAGroup(short _groupSize)
	throws BindingComponentException, SubarchitectureProcessException {
	if (m_currentlyBuiltProxy == null) {
	    throw new RuntimeException("m_currentlyBuiltProxy == null!!!");
	}

	if (m_currentlyBuiltProxy.m_type.equals(BindingProxyType.GROUP)) {
	    // throw new BindingComponentException(
	    // "Attempting to make a group of a proxy that is already a group");
	    log("Warning:Attempting to make a group of a proxy that is already a group");
	}
	else {
	    Group pl = new Group();
	    pl.m_size = _groupSize;
	    pl.m_groupDetailsID = newDataID();
	    FeaturePointer ptr = addFeatureToCurrentProxy(pl);
	    BindingFeatures.details.GroupDetails groupdetails = new BindingFeatures.details.GroupDetails();
	    groupdetails.m_groupFeatureID = ptr.m_address;
	    groupdetails.m_groupProxyID = m_currentProxyID;
	    groupdetails.m_groupMemberProxyIDs = new String[0];
	    addToWorkingMemory(pl.m_groupDetailsID, getBindingSA(),
			       groupdetails, OperationMode.BLOCKING);
	}
	m_currentlyBuiltProxy.m_type = BindingProxyType.GROUP;

	if (m_currentlyBuiltProxy.m_type.equals(BindingProxyType.GROUP)) {
	    boolean has_group_feature = false;
	    for (int i = 0; i < m_currentlyBuiltProxy.m_proxyFeatures.length; i++) {
		// if(m_currentlyBuiltProxy.m_proxyFeatures[i].m_type.equals(BindingOntology.GROUP_TYPE))
		// {
		if (m_currentlyBuiltProxy.m_proxyFeatures[i].m_type
		    .equals(CASTUtils.typeName(Group.class))) {
		    has_group_feature = true;
		}
	    }
	    if (!has_group_feature) {
		throw new RuntimeException(
					   "After just having added the bloody group feature: Group feature not in group proxy: "
					   + m_currentProxyID);
	    }
	}
    }

    protected BindingProxy deepCopy(BindingProxy _in) {

	if (_in == null) {
	    throw new RuntimeException("m_currentlyBuiltProxy == null!!!");
	}
	if (_in.m_proxyFeatures == null) {
	    throw new RuntimeException(
				       "m_currentlyBuiltProxy.m_proxyFeatures == null!!!");
	}
	if (_in.m_outPorts == null) {
	    throw new RuntimeException(
				       "m_currentlyBuiltProxy.m_outPorts == null!!!");
	}

	BindingProxy out = new BindingProxy(
					    new FeaturePointer[_in.m_proxyFeatures.length],
					    _in.m_featureSignature, _in.m_unionID, _in.m_proxyIDs,
					    _in.m_sourceID, _in.m_bestUnionsForProxyID,
					    _in.m_nonMatchingUnionID, _in.m_type, new ProxyPorts(
												 _in.m_outPorts.m_ownerProxyID,
												 new ProxyPort[_in.m_outPorts.m_ports.length]),
					    _in.m_inPortsID, _in.m_hypothetical, _in.m_updates,
					    _in.m_bindingCount, _in.m_proxyState);

	System.arraycopy(_in.m_proxyFeatures, 0, out.m_proxyFeatures, 0,
			 _in.m_proxyFeatures.length);
	System.arraycopy(_in.m_outPorts.m_ports, 0, out.m_outPorts.m_ports, 0,
			 _in.m_outPorts.m_ports.length);

	if (out.m_type.equals(BindingProxyType.GROUP)) {
	    boolean has_group_feature = false;
	    for (int i = 0; i < out.m_proxyFeatures.length; i++) {
		if (out.m_proxyFeatures[i].m_type.equals(CASTUtils
							 .typeName(Group.class))) {
		    has_group_feature = true;
		}
	    }
	    if (!has_group_feature) {
		throw new RuntimeException(
					   "At deepcopy: Group feature not in copied proxy...");
	    }
	}

	return out;
    }

    /**
     * stores the current proxy onto WM and returns the adress. It also resets
     * m_currentlyBuiltProxy to null.
     */
    protected String storeCurrentProxy() throws SubarchitectureProcessException {

	if (m_updatingProxy == false) {

	    // 1st, store the SourceID since it's required. The inherited
	    // method from AbstractMonitor can be used.
	    addSourceIDToCurrentProxy();
	    // Another simple thing to add is the creation time. Very
	    // useful for debugging. Could be used to eliminate "old"
	    // proxies too.
	    addCreationTimeToCurrentProxy();
	}

	if (m_updatingProxy == false) {
	    ThisProxyID thisProxyID = new ThisProxyID();
	    thisProxyID.m_thisProxyID = m_currentProxyID;
	    thisProxyID.m_parent = defaultParentFeature();
	    addFeatureToCurrentProxy(thisProxyID);

	    if (m_temporalFrameOfCurrentProxy != TemporalFrameType.NA) {
		TemporalFrame frame = new TemporalFrame();
		frame.m_temporalFrame = m_temporalFrameOfCurrentProxy;
		thisProxyID.m_parent = defaultParentFeature();
		addFeatureToCurrentProxy(frame);
	    }
	}

	if (m_currentlyBuiltProxy.m_type
	    .equals(CASTUtils.typeName(Group.class))) {
	    boolean has_group_feature = false;
	    for (int i = 0; i < m_currentlyBuiltProxy.m_proxyFeatures.length; i++) {
		if (m_currentlyBuiltProxy.m_proxyFeatures[i].m_type
		    .equals(CASTUtils.typeName(Group.class))) {
		    has_group_feature = true;
		}
	    }
	    if (!has_group_feature) {
		throw new RuntimeException(
					   "at store current proxy: Group feature not in group proxy: "
					   + m_currentProxyID);
	    }
	}

	// store proxy on WM

	// BindingProxy proxy_ptr =
	// deepCopy(m_currentlyBuiltProxy);

	TreeSet<String> featureIDs = new TreeSet<String>();
	for (int i = 0; i < m_currentlyBuiltProxy.m_proxyFeatures.length; i++) {
	    featureIDs.add(m_currentlyBuiltProxy.m_proxyFeatures[i].m_address);
	}
	String signature = new String();
	for (String str : featureIDs) {
	    signature += str;
	}
	m_currentlyBuiltProxy.m_featureSignature = signature;

	if (m_updatingProxy == false) {
	    log("added proxy: " + m_currentProxyID);
	    m_currentlyBuiltProxy.m_proxyState = BindingProxyState.NEW;
	    addToWorkingMemory(m_currentProxyID, getBindingSA(),
			       m_currentlyBuiltProxy, OperationMode.BLOCKING);
	    // sleepProcess(200);
	}
	else {
	    log("overwritten proxy: " + m_currentProxyID);
	    m_currentlyBuiltProxy.m_proxyState = BindingProxyState.UPDATED;
	    // assert(haveLatestVersion(m_currentProxyID, getBindingSA()));
	    getWorkingMemoryEntry(m_currentProxyID, getBindingSA());

	    overwriteWorkingMemory(m_currentProxyID, getBindingSA(),
				   m_currentlyBuiltProxy, OperationMode.BLOCKING);
	    // sleepProcess(200);

	}

	m_proxyID2inportsID.put(m_currentProxyID,
				m_currentlyBuiltProxy.m_inPortsID);

	// log(string("Current proxy feature count: ") +
	// lexical_cast<string>(m_currentlyBuiltProxy->m_proxyFeatures.length()));

	// update the inports, if any
	if (m_currentlyBuiltProxy.m_outPorts.m_ports.length > 0) {
	    updateInports(m_currentlyBuiltProxy.m_outPorts);
	}
	m_currentlyBuiltProxy = null;

	m_unboundProxyAddresses.add(m_currentProxyID);

	String addr = m_currentProxyID;

	m_currentProxyID = "";

	//if (m_updatingProxy == true) {
	bindNewProxies();
	//}

	return addr;

    }

    @Override
	public void start() {
	super.start();
	m_startCalled = true;
	try {
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
								       BindingData.BinderToken.class, WorkingMemoryOperation.ADD),
			    new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(
								 WorkingMemoryChange _wmc) {
				    binderReadySignal(_wmc);
				}
			    });
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
								       BindingQueries.MakeProxyUnavailable.class,
								       WorkingMemoryOperation.ADD),
			    new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(
								 WorkingMemoryChange _wmc) {
				    makeProxyUnavailableAdded(_wmc);
				}
			    });
	}
	catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    System.exit(1);
	}
    }

    // ! just sets m_binderReady to true
    public void binderReadySignal(WorkingMemoryChange _wmc) {
	m_binderReady = true;
    }

    // ! returns m_binderReady
    public boolean binderReady() {
	return m_binderReady;
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
     */
    @Override
	public void configure(Properties _config) {
	super.configure(_config);
	
	if(_config.containsKey(BINDING_SUBARCH_NOCHECK_CONFIG_KEY.value)) {
		String saList = _config.getProperty(BINDING_SUBARCH_NOCHECK_CONFIG_KEY.value);
		String[] sas = saList.split(",");
		for (String sa : sas) {
			log("not checking for unions in: " + sa);
			m_uncheckedSAs.add(sa);
		}
	}
	
    }

    /**
     * causes the proxy to be hypothetical, and will not be bound to anything.
     */
    protected void makeCurrentProxyHypothetical()
	throws BindingComponentException {
	if (m_currentlyBuiltProxy == null) {
	    throw new RuntimeException(
				       "cannot make a current proxy hypothetical if not loaded or started");
	}
	if (m_updatingProxy == true) {
	    throw new RuntimeException(
				       "An existing proxy can not be made hypothetical");
	}
	m_currentlyBuiltProxy.m_hypothetical = true;
    }

    /**
     * changes the temporal frame of the current proxy
     */
    protected void changeTemporalFrameOfCurrentProxy(TemporalFrameType _frame)
	throws BindingComponentException {
	if (m_currentlyBuiltProxy == null) {
	    throw new BindingComponentException(
						"cannot change temporal frame of a proxy which is not loaded or started");
	}
	m_temporalFrameOfCurrentProxy = _frame;
    }

    /**
     * starts a new proxy (OBS, it overwrites information if you havent stored
     * the last proxy)
     * 
     * @throws BindingComponentException
     */
    private void _startNewProxy(BindingProxyType _proxyType)
	throws BindingComponentException, SubarchitectureProcessException {
	if (!m_startCalled) {
	    throw new BindingComponentException(
						"error in binding monitor, probably you forgot to call super.start() in your monitor");
	}
	if (!binderReady()) {
	    throw new BindingComponentException(
						"error in binding monitor, you started a proxy before binderReady() was true. Perhaps delay a bit while things start up?");
	}

	if (m_currentlyBuiltProxy != null) {
	    throw new BindingComponentException(
						"A new proxy can not be started until the previous one is stored or cancelled.");
	}

	m_currentProxyID = newDataID();

	m_updatingProxy = false;

	m_currentlyBuiltProxy = new BindingProxy(new FeaturePointer[0], "", "",
						 new String[0], m_subarchitectureID, "", "", _proxyType,
						 new ProxyPorts(m_currentProxyID, new ProxyPort[0]), "", false,
						 0, 0, BindingProxyState.NEW);

	// just to be sure...
	m_currentlyBuiltProxy.m_unionID = "";
	String bestID = new String(newDataID());
	BestUnionsForProxy new_best = new BestUnionsForProxy();
	new_best.m_proxyID = m_currentProxyID;
	new_best.m_proxyFeatureSignature = "";
	new_best.m_unionIDs = new String[0];
	// now this version works
	new_best.m_score = new BindingScore(false, true, 0, true, 0, -1,
					    1.0E13, -1, "", "", "");
	new_best.m_proxyUpdatesWhenThisComputed = -1;

	addToWorkingMemory(bestID, getBindingSA(), new_best,
			   OperationMode.BLOCKING);

	String nonmatchingID = new String(newDataID());
	NonMatchingUnions new_nonmatching = new NonMatchingUnions();
	new_nonmatching.m_proxyID = m_currentProxyID;
	new_nonmatching.m_nonMatchingUnionIDs = new String[0];
	addToWorkingMemory(nonmatchingID, getBindingSA(), new_nonmatching,
			   OperationMode.BLOCKING);

	m_currentlyBuiltProxy.m_bestUnionsForProxyID = bestID;
	m_currentlyBuiltProxy.m_nonMatchingUnionID = nonmatchingID;

	m_currentlyBuiltProxy.m_type = _proxyType;
	m_currentlyBuiltProxy.m_hypothetical = false;

	String inPortsID = newDataID();

	ProxyPorts inPorts = new ProxyPorts(m_currentProxyID, new ProxyPort[0]);

	addToWorkingMemory(inPortsID, getBindingSA(), inPorts,
			   OperationMode.BLOCKING);

	// println("\n\n\ninPortsID: " + inPortsID);

	m_currentlyBuiltProxy.m_inPortsID = inPortsID;

	m_temporalFrameOfCurrentProxy = TemporalFrameType.NA;

    }

    /**
     * deletes one feature from the current proxy (suitable for when updating
     * proxies and only one feature should be updated)
     */
    public void deleteFeatureFromCurrentProxy(FeaturePointer _ptr)
	throws BindingComponentException, SubarchitectureProcessException {
	if (m_currentlyBuiltProxy == null) {
	    throw (new BindingComponentException(
						 "Attempt to delete feature from m_proxyFeatures before initiating it with startNewProxy"));
	}
	FeaturePointerSet deleted = new FeaturePointerSet();
	int j = 0;
	FeaturePointer[] newFeats = new FeaturePointer[m_currentlyBuiltProxy.m_proxyFeatures.length - 1];
	for (int i = 0; i < m_currentlyBuiltProxy.m_proxyFeatures.length; ++i) {
	    if (m_currentlyBuiltProxy.m_proxyFeatures[i].m_address
		.equals(_ptr.m_address)) {
		deleted.add(_ptr);
	    }
	    else {
		if (j == m_currentlyBuiltProxy.m_proxyFeatures.length - 1)
		    throw (new BindingComponentException(
							 "Attempting to delete a feature that was not on the proxy"));
		newFeats[j] = m_currentlyBuiltProxy.m_proxyFeatures[i];
		j++;
	    }
	}
	m_currentlyBuiltProxy.m_proxyFeatures = newFeats;
	_deleteFeatures(deleted);
	if (m_currentlyBuiltProxy.m_type.equals(BindingProxyType.GROUP)) {
	    boolean has_group_feature = false;
	    for (int i = 0; i < m_currentlyBuiltProxy.m_proxyFeatures.length; i++) {
		if (m_currentlyBuiltProxy.m_proxyFeatures[i].m_type
		    .equals(CASTUtils.typeName(Group.class))) {
		    has_group_feature = true;
		}
	    }
	    if (!has_group_feature) {
		throw new RuntimeException(
					   "After explicitly deleting a feature of type "
					   + _ptr.m_type
					   + ": Group feature not in group proxy: "
					   + m_currentProxyID);
	    }
	}

    }

    /**
     * a simplified way to create a BALTTime from a double
     */
    public BALTTime baltTime(double _t) {
	BALTTime ret = new BALTTime((int) _t, (int) ((_t - (int) _t) * 1.0E6));
	return ret;
    }
    /**
     * returns a start time in the infinite past (for open intervals)
     */
    public StartTime infinitePast() {
	StartTime ret = new StartTime(new BALTTime[0]);
	return ret;
    }
    /**
     * creates a start time at the specified instant
     */
    public StartTime startTime(BALTTime _t) {
	StartTime ret = new StartTime(new BALTTime[1]);
	ret.m_t[0] = _t;
	return ret;
    }
    /**
     * returns an end time in the infinite past (for open intervals)
     */
    public EndTime infiniteFuture() {
	EndTime ret = new EndTime(new BALTTime[0]);
	return ret;
    }
    /**
     * creates an end time at the specified instant
     */
    public EndTime endTime(BALTTime _t) {
	EndTime ret = new EndTime(new BALTTime[1]);
	ret.m_t[0] = _t;
	return ret;
    }

    /**
     * add saliency feature in the interval between \p _start and \p _end
     */
    public FeaturePointer addSalienceToCurrentProxy(StartTime _start,
						    EndTime _end) throws BindingComponentException,
									 SubarchitectureProcessException {
	Salience salience = new Salience();
	salience.m_start = _start;
	salience.m_end = _end;
	return addFeatureToCurrentProxy(salience);
    }

    /**
     * Adds a single time instant as salience time (for utterances, when words
     * "this is" are uttered, for example)
     */
    public FeaturePointer addSalienceToCurrentProxy(BALTTime _time)
	throws BindingComponentException, SubarchitectureProcessException {
	return addSalienceToCurrentProxy(startTime(_time), endTime(_time));
    }
    /**
     * Adds the current time as a salience time instant
     */
    public FeaturePointer addSalienceToCurrentProxy()
	throws BindingComponentException, SubarchitectureProcessException {
	return addSalienceToCurrentProxy(NativeProcessLauncher.getBALTTime());
    }

    private final void acquireToken(WorkingMemoryAddress _wma)
	throws SubarchitectureProcessException {
	debug("AbstractMonitor.acquireToken(): acquiring "
	      + CASTUtils.toString(_wma));
	lockEntry(_wma, WorkingMemoryPermissions.LOCKED_ODR);
	m_tokens.add(_wma);
	debug("AbstractMonitor.acquireToken(): acquired "
	      + CASTUtils.toString(_wma));
    }

    private final void releaseToken(WorkingMemoryAddress _wma) {
	try {
	    unlockEntry(_wma);
	    m_tokens.remove(_wma);
	    debug("AbstractMonitor.releaseToken() released: "
		  + CASTUtils.toString(_wma));
	}
	catch (cast.architecture.subarchitecture.WMException _e) {
	    _e.printStackTrace();
	    System.exit(-1);
	}
    }

    // ! the token for writing a BindTheseProxies
    private final void acquireBindTheseProxiesMonitorToken() {
	try {
	    acquireToken(_bindTheseProxiesMonitorTokenAddress());
	}
	catch (binding.common.BindingComponentException _e) {
	    _e.printStackTrace();
	    System.exit(-1);
	}
	catch (cast.architecture.subarchitecture.SubarchitectureProcessException _e) {
	    _e.printStackTrace();
	    System.exit(-1);
	}
    }

    private final void releaseBindTheseProxiesMonitorToken() {
	try {
	    releaseToken(_bindTheseProxiesMonitorTokenAddress());
	}
	catch (binding.common.BindingComponentException _e) {
	    _e.printStackTrace();
	    System.exit(-1);
	}
    }

    private final boolean hasBindTheseProxiesMonitorToken()
	throws BindingComponentException {
	return hasToken(_bindTheseProxiesMonitorTokenAddress());
    }

    private final WorkingMemoryAddress _bindTheseProxiesMonitorTokenAddress()
	throws BindingComponentException {
	WorkingMemoryAddress a = new WorkingMemoryAddress();
	a.m_subarchitecture = getBindingSA();
	a.m_id = bindTheseProxiesMonitorTokenID.value;
	return a;
    }

    // // ! the token for writing a updating proxies
    // private void acquireUpdateProxyMonitorToken() {
    // try {
    // acquireToken(_updateProxyMonitorTokenAdress());
    // m_hasUpdateToken = true;
    // }
    // catch (binding.common.BindingComponentException _e) {
    // _e.printStackTrace();
    // System.exit(-1);
    // }
    // catch (cast.architecture.subarchitecture.SubarchitectureProcessException
    // _e) {
    // _e.printStackTrace();
    // System.exit(-1);
    // }
    // }

    // private void releaseUpdateProxyMonitorToken() {
    // try {
    // releaseToken(_updateProxyMonitorTokenAdress());
    // m_hasUpdateToken = false;
    // }
    // catch (binding.common.BindingComponentException _e) {
    // _e.printStackTrace();
    // System.exit(-1);
    // }
    // }

    // private WorkingMemoryAddress _updateProxyMonitorTokenAdress()
    // throws BindingComponentException {
    // WorkingMemoryAddress a = new WorkingMemoryAddress();
    // a.m_subarchitecture = getBindingSA();
    // a.m_id = updateProxyMonitorTokenID.value;
    // return a;
    // }

    // ! the token for writing important stuff to binding WM
    private void acquireBinderToken() {
	try {
	    acquireToken(_binderTokenAddress());
	}
	catch (binding.common.BindingComponentException _e) {
	    _e.printStackTrace();
	    System.exit(-1);
	}
	catch (cast.architecture.subarchitecture.SubarchitectureProcessException _e) {
	    _e.printStackTrace();
	    System.exit(-1);
	}
    }

    private void releaseBinderToken() {
	try {
	    releaseToken(_binderTokenAddress());
	}
	catch (binding.common.BindingComponentException _e) {
	    _e.printStackTrace();
	    System.exit(-1);
	}
    }

    private final boolean hasBinderToken() throws BindingComponentException {
    	return hasToken(_binderTokenAddress());
    }

    private final boolean hasToken(WorkingMemoryAddress _address) {
    	for (WorkingMemoryAddress token : m_tokens) {
    		int res = m_tokens.comparator().compare(token, _address);
    		//if they match return true
    		if(res == 0) {
//    			System.out.println("returning true for: " + CASTUtils.toString(_address));
//    				System.out.println("is: " + token);
    			return true;
    		}
    		//if the input address is before this then we've gone too far
    		else if(res > 0) {
//    			System.out.println("returning false for: " + CASTUtils.toString(_address));
//    			for (WorkingMemoryAddress pt : m_tokens) {
//    				System.out.println("is not: " + CASTUtils.toString(pt));
//				}

    			return false;
    		}
    		
		}
    	
//		System.out.println("returning false for: " + CASTUtils.toString(_address));
//		for (WorkingMemoryAddress pt : m_tokens) {
//			System.out.println("is not: " + CASTUtils.toString(pt));
//		}    	
    	return false;
	}

	private WorkingMemoryAddress _binderTokenAddress()
	throws BindingComponentException {
	WorkingMemoryAddress a = new WorkingMemoryAddress();
	a.m_subarchitecture = getBindingSA();
	a.m_id = binderTokenID.value;
	return a;
    }

    // private boolean m_hasUpdateToken;

    // ! returns \p m_ownerProxyID
    public HashSet<String> ownedProxyIDs() {
	return m_ownedProxyIDs;
    }

    // / for dealing with when \p BindingQueries::MakeProxyUnavailable
    // / are added to WM. Calls \pmakeProxyUnavailable() if the entry
    // / belongs to this monitor;
    private void makeProxyUnavailableAdded(WorkingMemoryChange _wmc) {
	try {
	    log("makeProxyUnavailableAdded");
	    MakeProxyUnavailable unavail;
	    try {
		CASTData tmp = getWorkingMemoryEntry(_wmc.m_address);
		unavail = (MakeProxyUnavailable) tmp.getData();
	    }
	    catch (DoesNotExistOnWMException _e) {
		// noop. this is ok... it was someone else's struct
		return;
	    }
	    String proxyID = unavail.m_proxyID;
	    if (!m_ownedProxyIDs.contains(proxyID)) // not my proxy, ignore
		return;
	    // ok, it is my proxy, deal with it:
	    // 1st, delete the entry...
	    deleteFromWorkingMemory(_wmc.m_address);
	    // 2nd, deal with the entry
	    try {
		makeProxyUnavailable(unavail);
	    }
	    catch (DoesNotExistOnWMException _e) {
		if (proxyID.equals(_e.address().m_id)) // ah, it was our proxy
		    // that was gone... it
		    // was already deleted,
		    // no probs
		    /* noop */;
		else
		    // not our proxy, it was sth else that was rotten
		    throw (_e);
	    }
	}
	catch (Exception _e) {
	    _e.printStackTrace();
	    System.exit(-1);
	}
    }

    // / called by \p makeProxyUnavailableAdded if the \p
    // / \p BindingQueries::MakeProxyUnavailable belongs to this monitor. It
    // will simply
    // / call \p deleteExistingProxy() right away.
    // / \remark override this function if immediate deletion is not appropriate
    protected void makeProxyUnavailable(MakeProxyUnavailable _makeProxyUnavailable)
	throws DoesNotExistOnWMException, SubarchitectureProcessException {
	deleteExistingProxy(_makeProxyUnavailable.m_proxyID);
    }

    @Override
	protected void setBindingSA(String _bindingSA) {
	super.setBindingSA(_bindingSA);
	assert (m_unboundProxyAddresses.isEmpty());
	assert (m_tokens.isEmpty());
    }

}
