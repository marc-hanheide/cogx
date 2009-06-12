/**
 * 
 */
package binding.abstr;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Properties;
import java.util.TreeSet;

import BindingData.BINDINGSUBARCHNOCHECKCONFIGKEY;
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
// import BindingFeaturesCommon.TruthValue;
import BindingQueries.MakeProxyUnavailable;
// import balt.corba.autogen.FrameworkBasics.BALTTime;
// import balt.jni.NativeProcessLauncher;
import binding.FeaturePointerSet;
import binding.common.BindingComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
// import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPermissions;
import cast.core.CASTUtils;
 import binding.common.WMAComparator;
import cast.core.CASTData;
/**
 * @author henrikj
 */
public abstract class AbstractMonitor extends AbstractBindingReader {
    
    // / features from an updated proxy must be deleted to avoid
    // / memory leaks
    private FeaturePointerSet featuresToDelete;

    /**
     * defines the ID of the monitored SA
     */
    protected String sourceID;

    /**
     * The ID of the latest added sourceID, must be added to OtherSorceID so
     * that the scorer can avoid scoring it with a SourceID of the same proxy
     */
    protected String sourceIDAddress;

    /**
     * if currently built proxy is a new one, this is false
     */
    protected Boolean updatingProxy;
    /**
     * the ID of the build proxy.
     */
    protected String currentProxyID;
    /**
     * The proxy which is currently being built up by features being added to it
     * (and WM)
     */
    protected BindingProxy currentlyBuiltProxy;

    /*
     * A list of proxies added to the WM but which have not yet been bound
     * (bindNewProxies binds the proxies and empties the list)
     */
    protected HashSet<String> unboundProxyAddresses;


    /**
     * A list of subarchs not checked for union ids before updating
     */
    protected final HashSet<String> uncheckedSAs;

    
    // ! true iff memfun start() called
    private boolean startCalled;

    // ! true iff binderReadySignal has been called
    private boolean binderReady;

    /*
     * Maps from proxy IDs to the address of the inportlist
     */
    protected HashMap<String, String> proxyID2inportsID;

    // ! proxies that belong to this monitor
    private HashSet<String> ownedProxyIDs;

    private final TreeSet<WorkingMemoryAddress> tokens;

    /*
     * 
     */
    protected TemporalFrameType temporalFrameOfCurrentProxy;

    protected AbstractMonitor(String _id) {
	super();
	unboundProxyAddresses = new HashSet<String>();
	ownedProxyIDs = new HashSet<String>();
	featuresToDelete = new FeaturePointerSet();
	currentProxyID = "";
	WorkingMemoryChangeQueueBehaviour queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	sourceIDAddress = null;
	proxyID2inportsID = new HashMap();
	temporalFrameOfCurrentProxy = TemporalFrameType.NA;
	startCalled = false;
	// hasUpdateToken = false;
	tokens = new TreeSet<WorkingMemoryAddress>(new WMAComparator());
	uncheckedSAs = new HashSet<String>(1);
    }

    /**
     * Deletes the features from WM. Only to be used when cancelling an unstored
     * proxy, therefore private
     * 
     * @throws SubarchitectureComponentException
     */
    private void _deleteFeatures(FeaturePointerSet _features)
	throws SubarchitectureComponentException {
	for (FeaturePointer pointer : _features) {
	    // deleteFromWorkingMemory(pointer.address);
	    log("Deleting feature task added for feature: " + pointer.address);
	    ExplicitFeatureDeletionTask del_task = new ExplicitFeatureDeletionTask();
	    del_task.featureID = pointer.address;
	    addToWorkingMemory(newDataID(), del_task);
	}
    }

    /**
     * factory for ParentFeatures
     */
    protected ParentFeature defaultParentFeature() {
	return new ParentFeature(true, currentProxyID);
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

   /** protected String featureToCASTFeatureTypeString(Object _obj) {
	return CASTUtils.typeName(_obj);
    } */
    
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
						   boolean _truthValue) throws SubarchitectureComponentException {
	FeaturePointer feature = new FeaturePointer();

	try {
	    feature.address = newDataID();
	//    feature.type = CASTUtils.typeName(_anyFeature);
	    // featureToCASTFeatureTypeString(_anyFeature);
	    feature.immediateProxyID = currentProxyID;

	    setParent(_anyFeature, new ParentFeature(_truthValue,
						     currentProxyID));

	    // log("storing feature: " + feature.address + " " +
	    // feature.type);
	    addToWorkingMemory(feature.address, (Ice.ObjectImpl) _anyFeature);
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
	    Field parentField = _anyFeature.getClass().getField("parent");
	    parentField.set(_anyFeature, _parent);
	}
	catch (Exception e) {
	    throw new BindingComponentException(
						"Error setting parent field reflectively for class: "
						+ _anyFeature.getClass() +  " " + e.getMessage());
	}
    }

    /**
     * Adds the creation time to the proxy
     */
    protected FeaturePointer addCreationTimeToCurrentProxy()
	throws BindingComponentException, SubarchitectureComponentException {
	CreationTime creationTime = new CreationTime();
	creationTime.creationTimelong = System.currentTimeMillis();
	FeaturePointer ret = addFeatureToCurrentProxy(creationTime,
						      true);
	return ret;
    }

    /**
     * Stores a feature to WM and adds the adress to the set of features
     * pointers of the proxy
     */
    protected FeaturePointer addFeatureToCurrentProxy(Object _feature,
						      boolean _truthValue) throws BindingComponentException,
										     SubarchitectureComponentException {
	if (currentlyBuiltProxy == null) {
	    throw (new BindingComponentException(
						 "Attempt to add feature to proxyFeatures before initiating it with startNewProxy"));
	}
	FeaturePointer ptr = storeFeature(_feature, _truthValue);
	currentlyBuiltProxy.proxyFeatures = addFeatureToArray(
								  currentlyBuiltProxy.proxyFeatures, ptr);
	return ptr;
    }

    /**
     * Stores a feature to WM and adds the adress to the set of features
     * pointers of the proxy
     */
    protected FeaturePointer addFeatureToCurrentProxy(Object _feature)
	throws BindingComponentException, SubarchitectureComponentException {
	return addFeatureToCurrentProxy(_feature, true);
    }

    // private stuff below this point...

    /**
     * Adds a relation from proxy on WM address ID _from to proxy on WM address
     * ID _to with _label to denote the type of relation. Returns the stored
     * relation proxy's ID.
     */
    protected String addSimpleRelation(String _from, String _to, String _label,
				       TemporalFrameType _frame) throws BindingComponentException,
									SubarchitectureComponentException {
	if (currentlyBuiltProxy != null) {
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
	// rel.from = _from;
	// rel.to = _to;
	// rel.labelstr = _label;
	//
	// // OBSOLETE SOON:
	// addToWorkingMemory(newDataID(), bindingSA,
	// BindingOntology.SIMPLE_RELATION_TYPE, rel);

	// Now, try to add relation as a proxy too:
	startNewRelationProxy();

	// as it's acting as a simple rel, make sure it doesn't bind
	// within subarch
	// addOtherSourceIDToCurrentProxy(sourceID, true);

	RelationLabel label = new RelationLabel();
	label.labelstr = _label;
	addFeatureToCurrentProxy(label);
	DebugString info = new DebugString();
	info.debugStringstr = "from: " + _from + " to: " + _to;
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
	if (currentlyBuiltProxy.type != BindingProxyType.RELATION) {
	    throw (new BindingComponentException(
						 "only add outports for relation proxies, please."));
	}
	ProxyPort port = new ProxyPort();
	port.type = PortType.PROXY; // for now... ok...
	port.label = _portLabel;
	port.proxyID = _proxyID;
	port.ownerProxyID = currentProxyID;
	ProxyPort[] new_outports = new ProxyPort[currentlyBuiltProxy.outPorts.ports.length + 1];
	System.arraycopy(currentlyBuiltProxy.outPorts.ports, 0,
			 new_outports, 0,
			 currentlyBuiltProxy.outPorts.ports.length);

	new_outports[currentlyBuiltProxy.outPorts.ports.length] = port;
	currentlyBuiltProxy.outPorts.ports = new_outports;
    }

    protected void updateInports(ProxyPorts _ports)// _proxyID, String
    // _relationID, PortType
    // _portType, String
    // _portLabel)
	throws BindingComponentException, SubarchitectureComponentException {
	return;
	// deprecated	for (int i = 0; i < _ports.ports.length; ++i) {
	// deprecated	    
	// deprecated	    String proxyID = _ports.ports[i].proxyID;
	// deprecated	    String inportsID = proxyID2inportsID.get(proxyID);
	// deprecated	    try {
	// deprecated		if (inportsID == null) {
	// deprecated		    CASTData froproxy_data = getWorkingMemoryEntry(proxyID,
	// deprecated								     getBindingSA());
	// deprecated		    BindingProxy froproxy = (BindingProxy) froproxy_data
	// deprecated			.getData();
	// deprecated		    inportsID = froproxy.inPortsID;
	// deprecated		    proxyID2inportsID.put(proxyID, inportsID);
	// deprecated		}
	// deprecated		
	// deprecated		CASTData inportsData = getWorkingMemoryEntry(inportsID,
	// deprecated							     getBindingSA());
	// deprecated		ProxyPorts inports = (ProxyPorts) inportsData.getData();
	// deprecated		
	// deprecated		ProxyPort[] new_inports = new ProxyPort[inports.ports.length + 1];
	// deprecated		System.arraycopy(inports.ports, 0, new_inports, 0,
	// deprecated				 inports.ports.length);
	// deprecated		new_inports[inports.ports.length] = 
	// deprecated		    new ProxyPort(
	// deprecated				  _ports.ports[i].type, _ports.ports[i].labelstr,
	// deprecated				  _ports.ports[i].ownerProxyID,
	// deprecated				  _ports.ports[i].proxyID);
	// deprecated		
	// deprecated		// assert(haveLatestVersion(inportsID, getBindingSA()));
	// deprecated		overwriteWorkingMemory(inportsID, 
	// deprecated				       getBindingSA(),
	// deprecated				       new ProxyPorts(inports.ownerProxyID, new_inports),
	// deprecated				       OperationMode.BLOCKING);
	// deprecated		
	// deprecated		unboundProxyAddresses.add(proxyID);
	// deprecated	    }
	// deprecated	    catch (DoesNotExistOnWMException _e) {
	// deprecated		log("could not update inports of " + proxyID + " : "
	// deprecated		    + _e.toString());
	// deprecated	    }
	// deprecated	}
    }

    /**
     * adds the source ID to the proxy. Make sure your instantiated monitor sets
     * sourceID first.
     */
    protected FeaturePointer addSourceIDToCurrentProxy()
	throws BindingComponentException, SubarchitectureComponentException {
	SourceID sourceID = new SourceID();
	if (sourceID == null) {
	    throw new BindingComponentException(
						"AbstractMonitor.sourceID not initialized. This should be done in the implemented monitor.");

	}
	sourceID.sourceAddress = sourceID.sourceAddress;
	sourceID.monitorID = getProcessIdentifier();
	FeaturePointer ret = addFeatureToCurrentProxy(sourceID);
	sourceIDAddress = ret.address;
	return ret;
    }

    /**
     * specifies that the proy should be scored base on unions stemming from a
     * particular subarch (or vice versa, if negated)
     */
    protected FeaturePointer addOtherSourceIDToCurrentProxy(String _id,
							    boolean _truthValue) throws BindingComponentException,
											   SubarchitectureComponentException {
	OtherSourceID otherSourceID = new OtherSourceID();

	otherSourceID.otherSourceAddress = _id;
	otherSourceID.parent = defaultParentFeature();
	// otherSourceID.thisSourceIDRef = sourceIDAddress;
	FeaturePointer ret = addFeatureToCurrentProxy(otherSourceID,
						      _truthValue);
	return ret;
    }

    /**
     * After all new proxies are added and relations between them defined it is
     * time to bind them. This stores a BindTheseProxies object onto the WM and
     * resets unboundProxyAdresses to an empty list
     */
    protected void bindNewProxies() throws SubarchitectureComponentException {
	if (0 != unboundProxyAddresses.size()) {

	    if (!hasBindTheseProxiesMonitorToken()) {
		acquireBindTheseProxiesMonitorToken();
	    }
	    if (!hasBinderToken()) {
		acquireBinderToken();
	    }
	    releaseBinderToken();

	    try {
		BindTheseProxies bindThese = new BindTheseProxies();
		bindThese.proxyIDs = new String[unboundProxyAddresses
						  .size()];

		// can't use an index in a hashset :(
		int i = 0;
		for (String unboundProxyAddress : unboundProxyAddresses) {
		    log("bindNewProxies:" + unboundProxyAddress);
		    // ownedProxyIDs.add(bindThese.proxyIDs[i]);
		    // nah: above line is bad bad bad
		    ownedProxyIDs.add(unboundProxyAddress);
		    bindThese.proxyIDs[i++] = unboundProxyAddress;
		}

		for (String pid : ownedProxyIDs) {
		    log("owned: " + pid);
		}

		addToWorkingMemory(newDataID(), getBindingSA(), bindThese);
		unboundProxyAddresses.clear();

		// now delete any features that have been replaced/removed
		_deleteFeatures(featuresToDelete);
		featuresToDelete.clear();
	    }
	    catch (SubarchitectureComponentException _e) {
		releaseBindTheseProxiesMonitorToken();
		// if (hasUpdateToken)
		// releaseUpdateProxyMonitorToken();
		throw _e;
	    }
	    releaseBindTheseProxiesMonitorToken();
	    // if (hasUpdateToken)
	    // releaseUpdateProxyMonitorToken();
	}
    }

    /**
     * Cancels the creation of a proxy, deletes all created features up to this
     * point and makes it possible to start a new proxy.
     * 
     * @throws SubarchitectureComponentException
     */
    protected void cancelCurrentProxy() throws SubarchitectureComponentException {

	if (currentlyBuiltProxy == null) {
	    return;
	}

	FeaturePointerSet deleteThese = new FeaturePointerSet();

	for (FeaturePointer pointer : currentlyBuiltProxy.proxyFeatures) {
	    deleteThese.add(pointer);
	}

	_deleteFeatures(deleteThese);

	currentlyBuiltProxy = null;
	sourceIDAddress = null;
    }

    /**
     * calls protected void changeExistingProxy(_proxyAddr, new HashSet<String>())
     * 
     * @param _proxyAddr
     * @throws SubarchitectureComponentException
     */
    protected void changeExistingProxy(String _proxyAddr)
	throws SubarchitectureComponentException, BindingComponentException {
	// log("in changeExistingProxy(String _proxyAddr)");
	changeExistingProxy(_proxyAddr, new HashSet<String>());

	/*
	 * if (currentlyBuiltProxy != null) { throw new
	 * BindingComponentException( "An existing proxy can not be loaded until
	 * the previous one is stored or cancelled"); } currentProxyID =
	 * _proxyAddr; updatingProxy = true; // get proxy to be changed
	 * BindingProxy proxy = null; proxy = getBindingProxy(_proxyAddr); if
	 * (proxy != null) { // create new binding proxy locally
	 * currentlyBuiltProxy = deepCopy(proxy); for (FeaturePointer pointer :
	 * currentlyBuiltProxy.proxyFeatures) { // println("changed proxy
	 * has feature: " + // pointer.type);
	 * 
	 * if (pointer.type .equals(BindingOntology.SOURCE_ID_TYPE))
	 * sourceIDAddress = pointer.address; } } else { // reset default
	 * address currentProxyID = ""; updatingProxy = false;
	 * 
	 * throw new BindingComponentException( "at changing proxy in abstract
	 * monitor: missing proxy at address: " + _proxyAddr); }
	 */
    }

    private <T extends Ice.Object> void disallow(Class<T> _class,
			      HashSet<String> _deleteTheseFeatures)
	throws BindingComponentException {
	if (_deleteTheseFeatures.contains(CASTUtils.typeName( _class))) {
	    throw new RuntimeException(
				       CASTUtils.typeName(_class)
				       + " must not be in _deleteTheseFeatures in changeExistingProxy");
	}
    } 

    
    private final boolean isCheckedSA(String _sa) {
    	return !uncheckedSAs.contains(_sa);
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
     * @throws SubarchitectureComponentException
     */
    protected void changeExistingProxy(String _proxyAddr,
				       HashSet<String> _deleteTheseFeatures)
	throws SubarchitectureComponentException, BindingComponentException {
	// log("in AbstractMonitor::changeExistingProxy(const
	// string& _proxyAddr, const set<string>&
	// _deleteTheseFeatureTypes)");

	if (currentlyBuiltProxy != null) {
	    throw new BindingComponentException(
						"An existing proxy can not be loaded until the previous one is stored or cancelled");
	}

	// if (!hasUpdateToken)
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

	currentProxyID = _proxyAddr;
	updatingProxy = true;

	BindingProxy proxy = getBindingProxy(_proxyAddr);

	if (isCheckedSA(getBindingSA())
			&& proxy.unionID.equals("") && // i.e. unbound proxy
	    !unboundProxyAddresses.contains(currentProxyID)) {// i.e.,
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
				       + currentProxyID);
	}
	

	if (proxy != null) {

	    currentlyBuiltProxy = deepCopy(proxy);

	    currentlyBuiltProxy.proxyFeatures = new FeaturePointer[0];
	    // then, store the existing features for deletion and copy
	    // features
	    // we want to delete
	    currentlyBuiltProxy.updates = proxy.updates + 1;
	    for (FeaturePointer pointer : proxy.proxyFeatures) {
		// if (pointer.type.equals(BindingOntology.SOURCE_ID_TYPE))
		if (pointer.type.equals(CASTUtils.typeName(SourceID.class)))
		    sourceIDAddress = pointer.address;

		if (_deleteTheseFeatures.contains(pointer.type)) {
		    // log("will not copy this feature when updating
		    // proxy: "
		    // + pointer.type);
		    featuresToDelete.add(pointer);
		}
		else { // if not deleted, then we need to add it to the new
		    // proxy
		    log("will copy this feature when updating proxy: "
			+ pointer.type);
		    currentlyBuiltProxy.proxyFeatures = addFeatureToArray(
									      currentlyBuiltProxy.proxyFeatures, pointer);
		}
	    }
	    log("Size of the feature array: ["
		+ currentlyBuiltProxy.proxyFeatures.length
		+ "] (original: [" + proxy.proxyFeatures.length
		+ "]) for proxy type [" + currentlyBuiltProxy.type
		+ "]");
	    if (currentlyBuiltProxy.type.value() == BindingProxyType._GROUP) {
		boolean has_group_feature = false;
		for (int i = 0; i < currentlyBuiltProxy.proxyFeatures.length; i++) {
		    log("Scanning feature: type ["
			+ currentlyBuiltProxy.proxyFeatures[i].type
			+ "]");
		    if (currentlyBuiltProxy.proxyFeatures[i].type
			.equals(CASTUtils.typeName(Group.class))) {
			has_group_feature = true;
		    }
		}
		if (!has_group_feature) {
		    throw new RuntimeException(
					       "After explicitly copying nonexcluded features: Group feature not in group proxy: "
					       + currentProxyID);
		}
	    }

	}
	else {
	    // reset default address
	    currentProxyID = "";
	    updatingProxy = false;
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
     * @throws SubarchitectureComponentException
     */
    protected void deleteExistingProxy(String _proxyAddr)
	throws SubarchitectureComponentException {

	log("deleteExistingProxy: " + _proxyAddr);
	if (currentProxyID.equals(_proxyAddr)) {
	    cancelCurrentProxy();
	    return;
	}

	assert (ownedProxyIDs.contains(_proxyAddr));

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
	    del_task.proxyID = _proxyAddr;

	    // just in case:
	    unboundProxyAddresses.remove(_proxyAddr);

	    addToWorkingMemory(newDataID(), getBindingSA(), del_task);

	    ownedProxyIDs.remove(_proxyAddr);

	    /*
	     * FeaturePointerSet deleteThese = new FeaturePointerSet(); for
	     * (FeaturePointer pointer : proxy.proxyFeatures) {
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
     * @throws SubarchitectureComponentException
     */
    protected BindingProxy getBindingProxy(String _proxyAddr)
	throws SubarchitectureComponentException {

	BindingProxy ibc = null;

	try {
	    CASTData<?> wme = getWorkingMemoryEntry(_proxyAddr, getBindingSA());
	    ibc = (BindingProxy) wme.getData();
	}
	catch (SubarchitectureComponentException e) {
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
					       SubarchitectureComponentException {
	_startNewProxy(BindingProxyType.BASIC);
	temporalFrameOfCurrentProxy = TemporalFrameType.NA;
    }

    /**
     * starts a new proxy (OBS, it overwrites information if you havent stored
     * the last proxy)
     * 
     * @throws BindingComponentException
     */
    protected void startNewRelationProxy() throws BindingComponentException,
						  SubarchitectureComponentException {
	_startNewProxy(BindingProxyType.RELATION);
	temporalFrameOfCurrentProxy = TemporalFrameType.PERCEIVED;
	if (currentlyBuiltProxy == null) {
	    throw new RuntimeException("currentlyBuiltProxy == null!!!");
	}

    }

    protected void startNewGroupProxy(short _groupSize)
	throws BindingComponentException, SubarchitectureComponentException {
	startNewBasicProxy();
	makeCurrentProxyAGroup(_groupSize);
	temporalFrameOfCurrentProxy = TemporalFrameType.NA;
	if (currentlyBuiltProxy == null) {
	    throw new RuntimeException("currentlyBuiltProxy == null!!!");
	}
    }

    protected void makeCurrentProxyAGroup(short _groupSize)
	throws BindingComponentException, SubarchitectureComponentException {
	if (currentlyBuiltProxy == null) {
	    throw new RuntimeException("currentlyBuiltProxy == null!!!");
	}

	if (currentlyBuiltProxy.type.equals(BindingProxyType.GROUP)) {
	    // throw new BindingComponentException(
	    // "Attempting to make a group of a proxy that is already a group");
	    log("Warning:Attempting to make a group of a proxy that is already a group");
	}
	else {
	    Group pl = new Group();
	    pl.size = _groupSize;
	    pl.groupDetailsID = newDataID();
	    FeaturePointer ptr = addFeatureToCurrentProxy(pl);
	    BindingFeatures.details.GroupDetails groupdetails = new BindingFeatures.details.GroupDetails();
	    groupdetails.groupFeatureID = ptr.address;
	    groupdetails.groupProxyID = currentProxyID;
	    groupdetails.groupMemberProxyAddrs = new String[0];
	    addToWorkingMemory(pl.groupDetailsID, getBindingSA(),
			       groupdetails);
	}
	currentlyBuiltProxy.type = BindingProxyType.GROUP;

	if (currentlyBuiltProxy.type.equals(BindingProxyType.GROUP)) {
	    boolean has_group_feature = false;
	    for (int i = 0; i < currentlyBuiltProxy.proxyFeatures.length; i++) {
		// if(currentlyBuiltProxy.proxyFeatures[i].type.equals(BindingOntology.GROUP_TYPE))
		// {
		if (currentlyBuiltProxy.proxyFeatures[i].type
		    .equals(CASTUtils.typeName(Group.class))) {
		    has_group_feature = true;
		}
	    }
	    if (!has_group_feature) {
		throw new RuntimeException(
					   "After just having added the bloody group feature: Group feature not in group proxy: "
					   + currentProxyID);
	    }
	}
    }

    protected BindingProxy deepCopy(BindingProxy _in) {

	if (_in == null) {
	    throw new RuntimeException("currentlyBuiltProxy == null!!!");
	}
	if (_in.proxyFeatures == null) {
	    throw new RuntimeException(
				       "currentlyBuiltProxy.proxyFeatures == null!!!");
	}
	if (_in.outPorts == null) {
	    throw new RuntimeException(
				       "currentlyBuiltProxy.outPorts == null!!!");
	}

	BindingProxy out = new BindingProxy(
					    new FeaturePointer[_in.proxyFeatures.length],
					    _in.featureSignature, _in.unionID, _in.proxyIDs,
					    _in.sourceID, _in.bestUnionsForProxyID,
					    _in.nonMatchingUnionID, _in.type, new ProxyPorts(
												 _in.outPorts.ownerProxyID,
												 new ProxyPort[_in.outPorts.ports.length]),
					    _in.inPortsID, _in.hypothetical, _in.updates,
					    _in.bindingCount, _in.proxyState);

	System.arraycopy(_in.proxyFeatures, 0, out.proxyFeatures, 0,
			 _in.proxyFeatures.length);
	System.arraycopy(_in.outPorts.ports, 0, out.outPorts.ports, 0,
			 _in.outPorts.ports.length);

	if (out.type.equals(BindingProxyType.GROUP)) {
	    boolean has_group_feature = false;
	    for (int i = 0; i < out.proxyFeatures.length; i++) {
		if (out.proxyFeatures[i].type.equals(CASTUtils
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
     * currentlyBuiltProxy to null.
     */
    protected String storeCurrentProxy() throws SubarchitectureComponentException {

	if (updatingProxy == false) {

	    // 1st, store the SourceID since it's required. The inherited
	    // method from AbstractMonitor can be used.
	    addSourceIDToCurrentProxy();
	    // Another simple thing to add is the creation time. Very
	    // useful for debugging. Could be used to eliminate "old"
	    // proxies too.
	    addCreationTimeToCurrentProxy();
	}

	if (updatingProxy == false) {
	    ThisProxyID thisProxyID = new ThisProxyID();
	    thisProxyID.thisProxyAddress = currentProxyID;
	    thisProxyID.parent = defaultParentFeature();
	    addFeatureToCurrentProxy(thisProxyID);

	    if (temporalFrameOfCurrentProxy != TemporalFrameType.NA) {
		TemporalFrame frame = new TemporalFrame();
		frame.temporalFrametype = temporalFrameOfCurrentProxy;
		thisProxyID.parent = defaultParentFeature();
		addFeatureToCurrentProxy(frame);
	    }
	}

	if (currentlyBuiltProxy.type
	    .equals(CASTUtils.typeName(Group.class))) {
	    boolean has_group_feature = false;
	    for (int i = 0; i < currentlyBuiltProxy.proxyFeatures.length; i++) {
		if (currentlyBuiltProxy.proxyFeatures[i].type
		    .equals(CASTUtils.typeName(Group.class))) {
		    has_group_feature = true;
		}
	    }
	    if (!has_group_feature) {
		throw new RuntimeException(
					   "at store current proxy: Group feature not in group proxy: "
					   + currentProxyID);
	    }
	}

	// store proxy on WM

	// BindingProxy proxy_ptr =
	// deepCopy(currentlyBuiltProxy);

	TreeSet<String> featureIDs = new TreeSet<String>();
	for (int i = 0; i < currentlyBuiltProxy.proxyFeatures.length; i++) {
	    featureIDs.add(currentlyBuiltProxy.proxyFeatures[i].address);
	}
	String signature = new String();
	for (String str : featureIDs) {
	    signature += str;
	}
	currentlyBuiltProxy.featureSignature = signature;

	if (updatingProxy == false) {
	    log("added proxy: " + currentProxyID);
	    currentlyBuiltProxy.proxyState = BindingProxyState.NEW;
	    addToWorkingMemory(currentProxyID, getBindingSA(),
			       currentlyBuiltProxy);
	    // sleepComponent(200);
	}
	else {
	    log("overwritten proxy: " + currentProxyID);
	    currentlyBuiltProxy.proxyState = BindingProxyState.UPDATED;
	    // assert(haveLatestVersion(currentProxyID, getBindingSA()));
	    getWorkingMemoryEntry(currentProxyID, getBindingSA());

	    overwriteWorkingMemory(currentProxyID, getBindingSA(),
				   currentlyBuiltProxy);
	    // sleepComponent(200);

	}

	proxyID2inportsID.put(currentProxyID,
				currentlyBuiltProxy.inPortsID);

	// log(string("Current proxy feature count: ") +
	// lexical_cast<string>(currentlyBuiltProxy->proxyFeatures.length()));

	// update the inports, if any
	if (currentlyBuiltProxy.outPorts.ports.length > 0) {
	    updateInports(currentlyBuiltProxy.outPorts);
	}
	currentlyBuiltProxy = null;

	unboundProxyAddresses.add(currentProxyID);

	String addr = currentProxyID;

	currentProxyID = "";

	//if (updatingProxy == true) {
	bindNewProxies();
	//}

	return addr;

    }

    @Override
	public void start() {
	super.start();
	startCalled = true;
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
	catch (Exception e) {
	    e.printStackTrace();
	    System.exit(1);
	}
    }

    // ! just sets binderReady to true
    public void binderReadySignal(WorkingMemoryChange _wmc) {
	binderReady = true;
    }

    // ! returns binderReady
    public boolean binderReady() {
	return binderReady;
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
     */
    @Override
	public void configure(Map<String, String> _config) {
	super.configure(_config);
	
	if(_config.containsKey(BINDINGSUBARCHNOCHECKCONFIGKEY.value)) {
		String saList = _config.get(BINDINGSUBARCHNOCHECKCONFIGKEY.value);
		String[] sas = saList.split(",");
		for (String sa : sas) {
			log("not checking for unions in: " + sa);
			uncheckedSAs.add(sa);
		}
	}
	
    }

    /**
     * causes the proxy to be hypothetical, and will not be bound to anything.
     */
    protected void makeCurrentProxyHypothetical()
	throws BindingComponentException {
	if (currentlyBuiltProxy == null) {
	    throw new RuntimeException(
				       "cannot make a current proxy hypothetical if not loaded or started");
	}
	if (updatingProxy == true) {
	    throw new RuntimeException(
				       "An existing proxy can not be made hypothetical");
	}
	currentlyBuiltProxy.hypothetical = true;
    }

    /**
     * changes the temporal frame of the current proxy
     */
    protected void changeTemporalFrameOfCurrentProxy(TemporalFrameType _frame)
	throws BindingComponentException {
	if (currentlyBuiltProxy == null) {
	    throw new BindingComponentException(
						"cannot change temporal frame of a proxy which is not loaded or started");
	}
	temporalFrameOfCurrentProxy = _frame;
    }

    /**
     * starts a new proxy (OBS, it overwrites information if you havent stored
     * the last proxy)
     * 
     * @throws BindingComponentException
     */
    private void _startNewProxy(BindingProxyType _proxyType)
	throws BindingComponentException, SubarchitectureComponentException {
	if (!startCalled) {
	    throw new BindingComponentException(
						"error in binding monitor, probably you forgot to call super.start() in your monitor");
	}
	if (!binderReady()) {
	    throw new BindingComponentException(
						"error in binding monitor, you started a proxy before binderReady() was true. Perhaps delay a bit while things start up?");
	}

	if (currentlyBuiltProxy != null) {
	    throw new BindingComponentException(
						"A new proxy can not be started until the previous one is stored or cancelled.");
	}

	currentProxyID = newDataID();

	updatingProxy = false;

	currentlyBuiltProxy = new BindingProxy(new FeaturePointer[0], "", "",
						 new String[0], getSubarchitectureID(), "", "", _proxyType,
						 new ProxyPorts(currentProxyID, new ProxyPort[0]), "", false,
						 0, 0, BindingProxyState.NEW);

	// just to be sure...
	currentlyBuiltProxy.unionID = "";
	String bestID = new String(newDataID());
	BestUnionsForProxy new_best = new BestUnionsForProxy();
	new_best.proxyID = currentProxyID;
	new_best.proxyFeatureSignature = "";
	new_best.unionIDs = new String[0];
	// now this version works
	new_best.score = new BindingScore(false, true, 0, true, 0, -1,
					    1.0E13, -1, "", "", "");
	new_best.proxyUpdatesWhenThisComputed = -1;

	addToWorkingMemory(bestID, getBindingSA(), new_best);

	String nonmatchingID = new String(newDataID());
	NonMatchingUnions new_nonmatching = new NonMatchingUnions();
	new_nonmatching.proxyID = currentProxyID;
	new_nonmatching.nonMatchingUnionIDs = new String[0];
	addToWorkingMemory(nonmatchingID, getBindingSA(), new_nonmatching);

	currentlyBuiltProxy.bestUnionsForProxyID = bestID;
	currentlyBuiltProxy.nonMatchingUnionID = nonmatchingID;

	currentlyBuiltProxy.type = _proxyType;
	currentlyBuiltProxy.hypothetical = false;

	String inPortsID = newDataID();

	ProxyPorts inPorts = new ProxyPorts(currentProxyID, new ProxyPort[0]);

	addToWorkingMemory(inPortsID, getBindingSA(), inPorts);

	// println("\n\n\ninPortsID: " + inPortsID);

	currentlyBuiltProxy.inPortsID = inPortsID;

	temporalFrameOfCurrentProxy = TemporalFrameType.NA;

    }

    /**
     * deletes one feature from the current proxy (suitable for when updating
     * proxies and only one feature should be updated)
     */
    public void deleteFeatureFromCurrentProxy(FeaturePointer _ptr)
	throws BindingComponentException, SubarchitectureComponentException {
	if (currentlyBuiltProxy == null) {
	    throw (new BindingComponentException(
						 "Attempt to delete feature from proxyFeatures before initiating it with startNewProxy"));
	}
	FeaturePointerSet deleted = new FeaturePointerSet();
	int j = 0;
	FeaturePointer[] newFeats = new FeaturePointer[currentlyBuiltProxy.proxyFeatures.length - 1];
	for (int i = 0; i < currentlyBuiltProxy.proxyFeatures.length; ++i) {
	    if (currentlyBuiltProxy.proxyFeatures[i].address
		.equals(_ptr.address)) {
		deleted.add(_ptr);
	    }
	    else {
		if (j == currentlyBuiltProxy.proxyFeatures.length - 1)
		    throw (new BindingComponentException(
							 "Attempting to delete a feature that was not on the proxy"));
		newFeats[j] = currentlyBuiltProxy.proxyFeatures[i];
		j++;
	    }
	}
	currentlyBuiltProxy.proxyFeatures = newFeats;
	_deleteFeatures(deleted);
	if (currentlyBuiltProxy.type.equals(BindingProxyType.GROUP)) {
	    boolean has_group_feature = false;
	    for (int i = 0; i < currentlyBuiltProxy.proxyFeatures.length; i++) {
		if (currentlyBuiltProxy.proxyFeatures[i].type
		    .equals(CASTUtils.typeName(Group.class))) {
		    has_group_feature = true;
		}
	    }
	    if (!has_group_feature) {
		throw new RuntimeException(
					   "After explicitly deleting a feature of type "
					   + _ptr.type
					   + ": Group feature not in group proxy: "
					   + currentProxyID);
	    }
	}

    }

    /**
     * a simplified way to create a BALTTime from a double
     */
    /** public long long(double _t) {
	long ret = (int) _t, (int) ((_t - (int) _t) * 1.0E6);
	return ret;
    } */
    
    /**
     * returns a start time in the infinite past (for open intervals)
     */
    public StartTime infinitePast() {
	StartTime ret = new StartTime(new long[0]);
	return ret;
    }
    /**
     * creates a start time at the specified instant
     */
    public StartTime startTime(long _t) {
	StartTime ret = new StartTime(new long[1]);
	ret.t[0] = _t;
	return ret;
    }
    /**
     * returns an end time in the infinite past (for open intervals)
     */
    public EndTime infiniteFuture() {
	EndTime ret = new EndTime(new long[0]);
	return ret;
    }
    /**
     * creates an end time at the specified instant
     */
    public EndTime endTime(long _t) {
	EndTime ret = new EndTime(new long[1]);
	ret.t[0] = _t;
	return ret;
    }

    /**
     * add saliency feature in the interval between \p _start and \p _end
     */
    public FeaturePointer addSalienceToCurrentProxy(StartTime _start,
						    EndTime _end) throws BindingComponentException,
									 SubarchitectureComponentException {
	Salience salience = new Salience();
	salience.start = _start;
	salience.end = _end;
	return addFeatureToCurrentProxy(salience);
    }

    /**
     * Adds a single time instant as salience time (for utterances, when words
     * "this is" are uttered, for example)
     */
    public FeaturePointer addSalienceToCurrentProxy(long _time)
	throws BindingComponentException, SubarchitectureComponentException {
	return addSalienceToCurrentProxy(startTime(_time), endTime(_time));
    }
    /**
     * Adds the current time as a salience time instant
     */
    public FeaturePointer addSalienceToCurrentProxy()
	throws BindingComponentException, SubarchitectureComponentException {
	return addSalienceToCurrentProxy(System.currentTimeMillis());
    }

    private final void acquireToken(WorkingMemoryAddress _wma)
	throws SubarchitectureComponentException {
	debug("AbstractMonitor.acquireToken(): acquiring "
	      + CASTUtils.toString(_wma));
	lockEntry(_wma, WorkingMemoryPermissions.LOCKEDODR);
	tokens.add(_wma);
	debug("AbstractMonitor.acquireToken(): acquired "
	      + CASTUtils.toString(_wma));
    }

    private final void releaseToken(WorkingMemoryAddress _wma) {
	try {
	    unlockEntry(_wma);
	    tokens.remove(_wma);
	    debug("AbstractMonitor.releaseToken() released: "
		  + CASTUtils.toString(_wma));
	}
	catch (Exception _e) {
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
	catch (cast.SubarchitectureComponentException _e) {
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
	a.subarchitecture = getBindingSA();
	a.id = bindTheseProxiesMonitorTokenID.value;
	return a;
    }

    // // ! the token for writing a updating proxies
    // private void acquireUpdateProxyMonitorToken() {
    // try {
    // acquireToken(_updateProxyMonitorTokenAdress());
    // hasUpdateToken = true;
    // }
    // catch (binding.common.BindingComponentException _e) {
    // _e.printStackTrace();
    // System.exit(-1);
    // }
    // catch (cast.SubarchitectureComponentException
    // _e) {
    // _e.printStackTrace();
    // System.exit(-1);
    // }
    // }

    // private void releaseUpdateProxyMonitorToken() {
    // try {
    // releaseToken(_updateProxyMonitorTokenAdress());
    // hasUpdateToken = false;
    // }
    // catch (binding.common.BindingComponentException _e) {
    // _e.printStackTrace();
    // System.exit(-1);
    // }
    // }

    // private WorkingMemoryAddress _updateProxyMonitorTokenAdress()
    // throws BindingComponentException {
    // WorkingMemoryAddress a = new WorkingMemoryAddress();
    // a.subarchitecture = getBindingSA();
    // a.id = updateProxyMonitorTokenID.value;
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
	catch (cast.SubarchitectureComponentException _e) {
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
    	for (WorkingMemoryAddress token : tokens) {
    		int res = tokens.comparator().compare(token, _address);
    		//if they match return true
    		if(res == 0) {
//    			System.out.println("returning true for: " + CASTUtils.toString(_address));
//    				System.out.println("is: " + token);
    			return true;
    		}
    		//if the input address is before this then we've gone too far
    		else if(res > 0) {
//    			System.out.println("returning false for: " + CASTUtils.toString(_address));
//    			for (WorkingMemoryAddress pt : tokens) {
//    				System.out.println("is not: " + CASTUtils.toString(pt));
//				}

    			return false;
    		}
    		
		}
    	
//		System.out.println("returning false for: " + CASTUtils.toString(_address));
//		for (WorkingMemoryAddress pt : tokens) {
//			System.out.println("is not: " + CASTUtils.toString(pt));
//		}    	
    	return false;
	}

	private WorkingMemoryAddress _binderTokenAddress()
	throws BindingComponentException {
	WorkingMemoryAddress a = new WorkingMemoryAddress();
	a.subarchitecture = getBindingSA();
	a.id = binderTokenID.value;
	return a;
    }

    // private boolean hasUpdateToken;

    // ! returns \p ownerProxyID
    public HashSet<String> ownedProxyIDs() {
	return ownedProxyIDs;
    }

    // / for dealing with when \p BindingQueries::MakeProxyUnavailable
    // / are added to WM. Calls \pmakeProxyUnavailable() if the entry
    // / belongs to this monitor;
    private void makeProxyUnavailableAdded(WorkingMemoryChange _wmc) {
	try {
	    log("makeProxyUnavailableAdded");
	    MakeProxyUnavailable unavail;
	    try {
		CASTData tmp = getWorkingMemoryEntry(_wmc.address);
		unavail = (MakeProxyUnavailable) tmp.getData();
	    }
	    catch (DoesNotExistOnWMException _e) {
		// noop. this is ok... it was someone else's struct
		return;
	    }
	    String proxyID = unavail.proxyID;
	    if (!ownedProxyIDs.contains(proxyID)) // not my proxy, ignore
		return;
	    // ok, it is my proxy, deal with it:
	    // 1st, delete the entry...
	    deleteFromWorkingMemory(_wmc.address);
	    // 2nd, deal with the entry
	    try {
		makeProxyUnavailable(unavail);
	    }
	    catch (DoesNotExistOnWMException _e) {
	//	if (proxyID.equals(_e.address().id)) // ah, it was our proxy
		    // that was gone... it
		    // was already deleted,
		    // no probs
		    /* noop */;
	//	else
		    // not our proxy, it was sth else that was rotten
	//	    throw (_e);
		    _e.printStackTrace();
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
	throws DoesNotExistOnWMException, SubarchitectureComponentException {
	deleteExistingProxy(_makeProxyUnavailable.proxyID);
    }

    @Override
	protected void setBindingSA(String _bindingSA) {
	super.setBindingSA(_bindingSA);
	assert (unboundProxyAddresses.isEmpty());
	assert (tokens.isEmpty());
    }

}
