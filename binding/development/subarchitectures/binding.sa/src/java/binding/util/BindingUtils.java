/**
 * 
 */
package binding.util;

import java.util.ArrayList;

import binding.abstr.AbstractBindingReader;
import BindingData.*;
import BindingFeatures.RelationLabel;
import binding.common.BindingComponentException;
import cast.architecture.abstr.WorkingMemoryReaderProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.core.data.CASTData;
import cast.core.CASTUtils;

/**
 * @author nah
 */
@SuppressWarnings("unchecked")
public class BindingUtils {

    /**
         * 
         */
    private static final String TO = "to";

    /**
         * 
         */
    private static final String FROM = "from";

    /**
         * @param _proxyPorts
         * @param _label
         * @return
         */
    private static ProxyPort getPortWithLabel(ProxyPorts _proxyPorts,
	    String _label) {
	for (ProxyPort port : _proxyPorts.m_ports) {
	    if (port.m_label.equals(_label)) {
		return port;
	    }
	}
	return null;
    }

    /**
         * Given a relation proxy, this method returns all proxies connected to
         * this relation with the given port label (usually "to" or "from" for
         * simple relations).
         * 
         * @param _abr
         * @param _label
         * @param _proxy
         * @return
         * @throws SubarchitectureProcessException
         */
    private static ArrayList<CASTData<BindingProxy>> getProxiesFromRelationViaLabeledPort(
	    AbstractBindingReader _abr, String _label, BindingProxy _proxy)
	    throws SubarchitectureProcessException {
	// not sure this is correct: in ports vs out ports
	return getProxiesFromRelationViaLabeledPort(_abr, _abr.getBindingSA(),
		_label, _proxy);
    }

    /**
         * Given a relation proxy, this method returns all proxies connected to
         * this relation with the given port label (usually "to" or "from" for
         * simple relations).
         * 
         * @param _abr
         * @param _label
         * @param _proxy
         * @return
         * @throws SubarchitectureProcessException
         */
    private static ArrayList<CASTData<BindingProxy>> getProxiesFromRelationViaLabeledPort(
	    WorkingMemoryReaderProcess _abr, String _bindingSA, String _label,
	    BindingProxy _proxy) throws SubarchitectureProcessException {
	// not sure this is correct: in ports vs out ports
	return getProxiesFromRelationViaLabeledPort(_abr, _bindingSA, _label,
		_proxy.m_outPorts.m_ports, _proxy.m_type);
    }

    /**
         * Given a relation union, this method returns all proxies connected to
         * this relation with the given port label (usually "to" or "from" for
         * simple relations).
         * 
         * @param _abr
         * @param _label
         * @param _union
         * @return
         * @throws SubarchitectureProcessException
         */
    private static ArrayList<CASTData<BindingProxy>> getProxiesFromRelationViaLabeledPort(
	    AbstractBindingReader _abr, String _label, BindingUnion _union)
	    throws SubarchitectureProcessException {

	return getProxiesFromRelationViaLabeledPort(_abr, _label,
		_union.m_inPorts.m_ports, _union.m_type);
    }

    /**
         * Given a set of ports from a relation structure, this method returns
         * all proxies connected to ports with the given label.
         * 
         * @param _abr
         * @param _label
         * @param _ports
         * @param _type
         * @return
         * @throws SubarchitectureProcessException
         */
    private static ArrayList<CASTData<BindingProxy>> getProxiesFromRelationViaLabeledPort(
	    AbstractBindingReader _abr, String _label, ProxyPort[] _ports,
	    BindingProxyType _type) throws SubarchitectureProcessException {
	return getProxiesFromRelationViaLabeledPort(_abr, _abr.getBindingSA(),
		_label, _ports, _type);
    }

    private static ArrayList<CASTData<BindingProxy>> getProxiesFromRelationViaLabeledPort(
	    WorkingMemoryReaderProcess _abr, String _bindingSA, String _label,
	    ProxyPort[] _ports, BindingProxyType _type)
	    throws SubarchitectureProcessException {

	assert (_type == BindingProxyType.RELATION);

	ArrayList<CASTData<BindingProxy>> proxies = new ArrayList<CASTData<BindingProxy>>();

	for (ProxyPort port : _ports) {
	    if (port.m_label.equals(_label)) {
		proxies.add(getProxyData(_abr, _bindingSA, port.m_proxyID));
	    }
	}

	return proxies;
    }

    public static String findFeature(BindingProxy _bindingProxy,
				     String _featureType) {
	return findFeature(_bindingProxy.m_proxyFeatures, _featureType);
    }
    
    public static String findFeature(BindingProxy _bindingProxy,
				     Class<?> _class) {	
	return findFeature(_bindingProxy, CASTUtils.typeName(_class));
    }

    public static String findFeature(BindingProxy _bindingProxy,
				     Object _object) {	
	return findFeature(_bindingProxy, CASTUtils.typeName(_object));
    }

    public static String findFeature(BindingUnion _bindingUnion,
				     String _featureType) {
	return findFeature(_bindingUnion.m_unionFeatures, _featureType);
    }

    public static String findFeature(BindingUnion _bindingUnion,
				     Class<?> _class) {	
	return findFeature(_bindingUnion, CASTUtils.typeName(_class));
    }

    public static String findFeature(BindingUnion _bindingUnion,
				     Object _object) {	
	return findFeature(_bindingUnion, CASTUtils.typeName(_object));
    }


    public static String findFeature(FeaturePointer[] _ptrs, String _type) {
	for (FeaturePointer featurePointer : _ptrs) {
	    if (featurePointer.m_type.equals(_type)) {
		return featurePointer.m_address;
	    }
	}
	return null;
    }

    public static ArrayList<String> findFeatures(BindingProxy _bindingProxy,
						 String _featureType) {
	return findFeatures(_bindingProxy.m_proxyFeatures, _featureType);
    }
    
    public static ArrayList<String> findFeatures(BindingUnion _bindingUnion,
						 String _featureType) {
	return findFeatures(_bindingUnion.m_unionFeatures, _featureType);
    }
    
    public static ArrayList<String> findFeatures(FeaturePointer[] _ptrs,
						 String _type) {
	ArrayList<String> ids = new ArrayList<String>();
	for (FeaturePointer featurePointer : _ptrs) {
	    if (featurePointer.m_type.equals(_type)) {
		ids.add(featurePointer.m_address);
	    }
	}
	return ids;
    }

    public static <T> T getBindingFeature(AbstractBindingReader _abr,
	    BindingProxy _proxy, Class<T> _featureClass)
	    throws SubarchitectureProcessException {
	return getBindingFeature(_abr, _abr.getBindingSA(), _proxy,
		_featureClass);
    }

    public static <T> T getBindingFeature(WorkingMemoryReaderProcess _abr,
					  String _bindingSA, 
					  BindingProxy _proxy, 
					  Class<T> _featureClass)
	throws SubarchitectureProcessException {
	String featureID = findFeature(_proxy, 
				       CASTUtils.typeName(_featureClass));
	T feature = null;
	if (featureID != null) {
	    feature = getObject(_abr, featureID, _bindingSA, _featureClass)
		    .getData();
	}
	return feature;
    }

    public static <T> T getBindingFeature(WorkingMemoryReaderProcess _abr,
	    String _bindingSA, BindingUnion _union, Class<T> _featureClass)
	    throws SubarchitectureProcessException {
	String featureID = findFeature(_union, 
				       CASTUtils.typeName(_featureClass));
	T feature = null;
	if (featureID != null) {
	    feature = getObject(_abr, featureID, _bindingSA, _featureClass)
		    .getData();
	}
	return feature;
    }

// deprecated    /**
// deprecated         * @param _visualLearningMotiveHandler
// deprecated         * @param _feature
// deprecated         * @return
// deprecated         * @throws CASTOntologyException
// deprecated         * @throws SubarchitectureProcessException
// deprecated         */
// deprecated    @SuppressWarnings("unchecked")
// deprecated    public static <T> T getBindingFeature(AbstractBindingReader _abr,
// deprecated	    FeaturePointer _fp) throws SubarchitectureProcessException,
// deprecated	    CASTOntologyException {
// deprecated	return (T) getObject(
// deprecated		_abr,
// deprecated		_fp.m_address,
// deprecated		_abr.getBindingSA(),
// deprecated		BindingOntologyFactory.getOntology().ontologicalTypeToDataType(
// deprecated			_fp.m_type)).getData();
// deprecated    }
// deprecated
// deprecated    public static <T> T getBindingFeature(WorkingMemoryReaderProcess _abr,
// deprecated	    String _bindingSA, FeaturePointer _fp)
// deprecated	    throws SubarchitectureProcessException, CASTOntologyException {
// deprecated
// deprecated	return (T) getObject(
// deprecated		_abr,
// deprecated		_fp.m_address,
// deprecated		_bindingSA,
// deprecated		BindingOntologyFactory.getOntology().ontologicalTypeToDataType(
// deprecated			_fp.m_type)).getData();
// deprecated    }

    /**
         * Get all features of the particular type.
         * 
         * @param _abr
         * @param _union
         * @param _cls
         * @return
         * @throws SubarchitectureProcessException
         */
    public static <T> ArrayList<T> getBindingFeatures(
	    AbstractBindingReader _abr, BindingUnion _union, Class<T> _cls)
	    throws SubarchitectureProcessException {

	return getBindingFeatures(_abr, _abr.getBindingSA(), _union, _cls);
    }

    /**
         * Get all features of the particular type.
         * 
         * @param _abr
         * @param _union
         * @param _cls
         * @return
         * @throws SubarchitectureProcessException
         */
    public static <T> ArrayList<T> getBindingFeatures(
	    WorkingMemoryReaderProcess _abr, String _bindingSA,
	    BindingUnion _union, Class<T> _cls)
	    throws SubarchitectureProcessException {

	ArrayList<String> ids = findFeatures(_union, CASTUtils.typeName(_cls));
	ArrayList<T> features = new ArrayList<T>();
	for (String string : ids) {
	    features.add(getObject(_abr, string, _bindingSA, _cls).getData());
	}
	return features;
    }
    
    public static <T> ArrayList<T> getBindingFeatures(
    		WorkingMemoryReaderProcess _abr, String _bindingSA, 
    		BindingProxy _proxy, Class<T> _cls) 
    		throws SubarchitectureProcessException {

    	ArrayList<String> ids = findFeatures(_proxy, CASTUtils.typeName(_cls));
    	ArrayList<T> features = new ArrayList<T>();
    	for (String string : ids) {
    		features.add(getObject(_abr, string, _bindingSA, _cls).getData());
    	}
    	return features;
    }


    public static ProxyPort getFromPort(BindingProxy _proxy) {
	return getPortWithLabel(_proxy.m_outPorts, BindingUtils.FROM);
    }

    /**
         * Simple method to pick out a rel from a list.
         * 
         * @param _string
         * @param _rels
         * @return
         * @throws SubarchitectureProcessException
         */
    public static BindingProxy getLabeledRelation(AbstractBindingReader _abr,
	    String _label, ArrayList<BindingProxy> _relations)
	    throws SubarchitectureProcessException {
	return getLabeledRelation(_abr, _abr.getBindingSA(), _label, _relations);
    }

    /**
         * Simple method to pick out a rel from a list.
         * 
         */
    public static BindingProxy getLabeledRelation(
	    WorkingMemoryReaderProcess _abr, String _bindingSA, String _label,
	    ArrayList<BindingProxy> _relations)
	    throws SubarchitectureProcessException {
	for (BindingProxy proxy : _relations) {
	    RelationLabel rl = getBindingFeature(_abr, _bindingSA, proxy,
		    RelationLabel.class);
	    if (rl != null) {
		if (rl.m_label.equals(_label)) {
		    return proxy;
		}
	    }
	}
	return null;
    }

    @SuppressWarnings("unchecked")
    public static <T> CASTData<T> getObject(WorkingMemoryReaderProcess _abr,
	    String _id, String _subarch, Class<T> _objCls)
	    throws SubarchitectureProcessException {

	T obj = null;
	CASTData<?> wme = _abr.getWorkingMemoryEntry(_id, _subarch);
	// sanity check
	if (wme.getData().getClass().equals(_objCls)) {
	    return (CASTData<T>) wme;
	}

	throw new SubarchitectureProcessException("Unable to cast wme of type "
		+ wme.getData().getClass() + " to " + _objCls);

    }

    /**
         * @param _objAddress
         * @return
         * @throws SubarchitectureProcessException
         */
    public static <T> CASTData<T> getObject(WorkingMemoryReaderProcess _abr,
	    WorkingMemoryAddress _objAddress, Class<T> _objCls)
	    throws SubarchitectureProcessException {
	return getObject(_abr, _objAddress.m_id, _objAddress.m_subarchitecture,
		_objCls);

    }

    /**
         * @param _objects
         * @param name
         * @return
         * @throws SubarchitectureProcessException
         */
    public static <T> ArrayList<CASTData<T>> getObjects(
	    AbstractBindingReader _abr,
	    WorkingMemoryAddress[] _objectAddresses, Class<T> _objCls)
	    throws SubarchitectureProcessException {

	ArrayList<CASTData<T>> objects = new ArrayList<CASTData<T>>(
		_objectAddresses.length);
	for (WorkingMemoryAddress address : _objectAddresses) {
	    objects.add(getObject(_abr, address, _objCls));
	}
	return objects;

    }

    // public static ArrayList<BindingProxy>
    // getProxiesViaFrom(AbstractBindingReader _abr,
    // BindingProxy _rel)
    // throws SubarchitectureProcessException {
    // return getProxiesViaLabel(_abr, _rel, BindingUtils.FROM);
    // }

    /**
         * Given a relation union, this method returns all proxies connected to
         * this relation via the relation's "from" port.
         * 
         * @param _abr
         * @param _union
         * @return
         * @throws SubarchitectureProcessException
         */
    public static ArrayList<CASTData<BindingProxy>> getProxiesViaFromPort(
	    AbstractBindingReader _abr, BindingProxy _proxy)
	    throws SubarchitectureProcessException {
	return getProxiesFromRelationViaLabeledPort(_abr, _abr.getBindingSA(),
		BindingUtils.FROM, _proxy);
    }

    public static ArrayList<CASTData<BindingProxy>> getProxiesViaFromPort(
	    WorkingMemoryReaderProcess _abr, String _bindingSA,
	    BindingProxy _proxy) throws SubarchitectureProcessException {
	return getProxiesFromRelationViaLabeledPort(_abr, _bindingSA,
		BindingUtils.FROM, _proxy);
    }

    /**
         * Given a relation union, this method returns all proxies connected to
         * this relation via the relation's "from" port.
         * 
         * @param _abr
         * @param _union
         * @return
         * @throws SubarchitectureProcessException
         */
    public static ArrayList<CASTData<BindingProxy>> getProxiesViaFromPort(
	    AbstractBindingReader _abr, BindingUnion _union)
	    throws SubarchitectureProcessException {
	return getProxiesFromRelationViaLabeledPort(_abr, BindingUtils.FROM,
		_union);
    }

    // public static ArrayList<BindingProxy>
    // getProxiesViaLabel(AbstractBindingReader _abr,
    // BindingProxy _rel,
    // String _label)
    // throws SubarchitectureProcessException {
    //
    // assert (_rel.m_type == BindingProxyType.RELATION);
    //
    // ArrayList<BindingProxy> proxies = new ArrayList<BindingProxy>();
    // ProxyPort[] outports = _rel.m_outPorts.m_ports;
    // for (ProxyPort proxyPort : outports) {
    // if (proxyPort.m_label.equals(_label)) {
    // proxies.add(getProxy(_abr, proxyPort.m_proxyID));
    // }
    // }
    //
    // return proxies;
    //
    // }

    /**
         * Given a relation proxy, this method returns all proxies connected to
         * this relation via the relation's "to" port.
         * 
         * @param _abr
         * @param _union
         * @return
         * @throws SubarchitectureProcessException
         */
    public static ArrayList<CASTData<BindingProxy>> getProxiesViaToPort(
	    AbstractBindingReader _abr, BindingProxy _proxy)
	    throws SubarchitectureProcessException {
	return getProxiesViaToPort(_abr, _abr.getBindingSA(), _proxy);
    }

    public static ArrayList<CASTData<BindingProxy>> getProxiesViaToPort(
	    WorkingMemoryReaderProcess _abr, String _bindingSA,
	    BindingProxy _proxy) throws SubarchitectureProcessException {
	return getProxiesFromRelationViaLabeledPort(_abr, _bindingSA,
		BindingUtils.TO, _proxy);
    }

    /**
         * Given a relation union, this method returns all proxies connected to
         * this relation via the relation's "to" port.
         * 
         * @param _abr
         * @param _union
         * @return
         * @throws SubarchitectureProcessException
         */
    public static ArrayList<CASTData<BindingProxy>> getProxiesViaToPort(
	    AbstractBindingReader _abr, BindingUnion _union)
	    throws SubarchitectureProcessException {
	return getProxiesFromRelationViaLabeledPort(_abr, BindingUtils.TO,
		_union);
    }

    public static BindingProxy getProxy(AbstractBindingReader _abr,
	    String _proxyID) throws SubarchitectureProcessException {
	return getProxy(_abr, _abr.getBindingSA(), _proxyID);
    }

    public static BindingProxy getProxy(WorkingMemoryReaderProcess _abr,
	    String _bindingSA, String _proxyID)
	    throws SubarchitectureProcessException {
	return BindingUtils.getObject(_abr, _proxyID, _bindingSA,
		BindingProxy.class).getData();
    }

    public static BindingProxy getProxy(AbstractBindingReader _abr,
	    WorkingMemoryAddress _proxyAddr)
	    throws SubarchitectureProcessException {
	return BindingUtils.getObject(_abr, _proxyAddr.m_id,
		_proxyAddr.m_subarchitecture, BindingProxy.class).getData();
    }

    public static CASTData<BindingProxy> getProxyData(
	    AbstractBindingReader _abr, String _proxyID)
	    throws SubarchitectureProcessException {
	return BindingUtils.getObject(_abr, _proxyID, _abr.getBindingSA(),
		BindingProxy.class);
    }

    public static CASTData<BindingProxy> getProxyData(
	    WorkingMemoryReaderProcess _abr, String _bindingSA, String _proxyID)
	    throws SubarchitectureProcessException {
	return BindingUtils.getObject(_abr, _proxyID, _bindingSA,
		BindingProxy.class);
    }

    /**
         * Gets all relations this proxy is involved in.
         * 
         * @param _abr
         * @param _proxy
         * @return
         * @throws SubarchitectureProcessException
         * @throws BindingComponentException
         */
    public static ArrayList<BindingProxy> getRelations(
	    AbstractBindingReader _abr, BindingProxy _proxy)
	    throws BindingComponentException, SubarchitectureProcessException {
	return getRelations(_abr, _abr.getBindingSA(), _proxy);
    }

    /**
         * Gets all relations this proxy is involved in.
         * 
         * @param _abr
         * @param _proxy
         * @return
         * @throws SubarchitectureProcessException
         * @throws BindingComponentException
         */
    public static ArrayList<BindingProxy> getRelations(
	    WorkingMemoryReaderProcess _abr, String _bindingSA,
	    BindingProxy _proxy) throws BindingComponentException,
	    SubarchitectureProcessException {

	// may change to groups too later
	assert (_proxy.m_type == BindingProxyType.BASIC);

	ProxyPorts inports = getObject(_abr, _proxy.m_inPortsID, _bindingSA,
		ProxyPorts.class).getData();
	ArrayList<BindingProxy> rels = new ArrayList<BindingProxy>();
	for (ProxyPort inport : inports.m_ports) {
	    // System.out.println("added proxy from: " +
	    // inport.m_label);
	    rels.add(getProxy(_abr, _bindingSA, inport.m_proxyID));
	}

	return rels;
    }

    /**
         * Gets all relations this union is involved in.
         * 
         * @param _abr
         * @param _proxy
         * @return
         * @throws SubarchitectureProcessException
         * @throws BindingComponentException
         */
    public static ArrayList<BindingProxy> getRelations(
	    WorkingMemoryReaderProcess _abr, String _bindingSA,
	    BindingUnion _union) throws BindingComponentException,
	    SubarchitectureProcessException {

	// may change to groups too later
	assert (_union.m_type == BindingProxyType.BASIC);

	ProxyPorts inports = _union.m_inPorts;
	ArrayList<BindingProxy> rels = new ArrayList<BindingProxy>();
	for (ProxyPort inport : inports.m_ports) {
	    // System.out.println("added proxy from: " +
	    // inport.m_label);
	    rels.add(getProxy(_abr, _bindingSA, inport.m_proxyID));
	}

	return rels;
    }

    public static ArrayList<BindingProxy> getRelationsViaLabeledPort(
	    WorkingMemoryReaderProcess _abr, String _bindingSA,
	    BindingUnion _union, String _label)
	    throws SubarchitectureProcessException {
	// may change to groups too later
	assert (_union.m_type == BindingProxyType.BASIC);

	ProxyPorts inports = _union.m_inPorts;
	ArrayList<BindingProxy> rels = new ArrayList<BindingProxy>();
	for (ProxyPort inport : inports.m_ports) {
	    if (inport.m_label.equals(_label)) {
		rels.add(getProxy(_abr, _bindingSA, inport.m_proxyID));
	    }
	}

	return rels;
    }

    public static ArrayList<BindingProxy> getRelationsFromUnion(
	    WorkingMemoryReaderProcess _abr, String _bindingSA,
	    BindingUnion _union) throws BindingComponentException,
	    SubarchitectureProcessException {
	return getRelationsViaLabeledPort(_abr, _bindingSA, _union, FROM);
    }

    public static ArrayList<BindingProxy> getRelationsToUnion(
	    WorkingMemoryReaderProcess _abr, String _bindingSA,
	    BindingUnion _union) throws BindingComponentException,
	    SubarchitectureProcessException {
	return getRelationsViaLabeledPort(_abr, _bindingSA, _union, TO);
    }

    /**
         * @param _abr
         * @param _landmarkUnion
         * @return
         * @throws SubarchitectureProcessException
         */
    public static ArrayList<BindingProxy> getRelations(
	    AbstractBindingReader _abr, BindingUnion _landmarkUnion)
	    throws SubarchitectureProcessException {
	ArrayList<BindingProxy> rels = new ArrayList<BindingProxy>();
	ProxyPort[] inports = _landmarkUnion.m_inPorts.m_ports;
	for (ProxyPort in : inports) {
	    rels.add(getProxy(_abr, in.m_proxyID));
	}

	return rels;
    }

    public static ArrayList<CASTData<BindingProxy>> getRelationsData(
	    AbstractBindingReader _abr, BindingUnion _landmarkUnion)
	    throws SubarchitectureProcessException {
	ArrayList<CASTData<BindingProxy>> rels = new ArrayList<CASTData<BindingProxy>>();
	ProxyPort[] inports = _landmarkUnion.m_inPorts.m_ports;
	for (ProxyPort in : inports) {
	    rels.add(getProxyData(_abr, in.m_proxyID));
	}

	return rels;
    }

    public static ArrayList<CASTData<BindingProxy>> getRelationsData(
	    AbstractBindingReader _abr, BindingProxy _proxy)
	    throws SubarchitectureProcessException {
	ArrayList<CASTData<BindingProxy>> rels = new ArrayList<CASTData<BindingProxy>>();
	// may change to groups too later
	assert (_proxy.m_type == BindingProxyType.BASIC);

	ProxyPorts inports = getObject(_abr, _proxy.m_inPortsID,
		_abr.getBindingSA(), ProxyPorts.class).getData();

	for (ProxyPort inport : inports.m_ports) {
	    // System.out.println("added proxy from: " +
	    // inport.m_label);
	    rels.add(getProxyData(_abr, _abr.getBindingSA(), inport.m_proxyID));
	}

	return rels;
    }

    public static ProxyPort getToPort(BindingProxy _proxy) {
	return getPortWithLabel(_proxy.m_outPorts, BindingUtils.TO);
    }

    public static BindingUnion getUnion(AbstractBindingReader _abr,
	    String _unionID) throws SubarchitectureProcessException {
	return BindingUtils.getObject(_abr, _unionID, _abr.getBindingSA(),
		BindingUnion.class).getData();
    }

    public static BindingUnion getUnion(AbstractBindingReader _abr,
	    BindingProxy _proxy) throws SubarchitectureProcessException {
	return BindingUtils.getObject(_abr, _proxy.m_unionID,
		_abr.getBindingSA(), BindingUnion.class).getData();
    }

    public static BindingUnion getUnion(WorkingMemoryReaderProcess _abr,
	    String _bindingSA, String _unionID)
	    throws SubarchitectureProcessException {
	return BindingUtils.getObject(_abr, _unionID, _bindingSA,
		BindingUnion.class).getData();
    }

    public static CASTData<BindingUnion> getUnionData(
	    AbstractBindingReader _abr, String _unionID)
	    throws SubarchitectureProcessException {
	return BindingUtils.getObject(_abr, _unionID, _abr.getBindingSA(),
		BindingUnion.class);
    }

    /**
         * @param _proxy
         * @return
         */
    public static final boolean isBasicProxy(BindingProxy _proxy) {
	return _proxy.m_type == BindingProxyType.BASIC;
    }

    /**
         * @param _proxy
         * @return
         */
    public static final boolean isRelationProxy(BindingProxy _proxy) {
	return _proxy.m_type == BindingProxyType.RELATION;
    }

    /**
         * Given a proxy, get the relation leading from it with the given
         * relation
         * 
         * @param _abr
         * @param _label
         * @param _proxy
         * @return
         * @throws SubarchitectureProcessException
         * @throws BindingComponentException
         */
    public static BindingProxy getLabeledRelation(AbstractBindingReader _abr,
	    String _label, BindingProxy _proxy)
	    throws BindingComponentException, SubarchitectureProcessException {
	return getLabeledRelation(_abr, _label, getRelations(_abr, _proxy));
    }

    /**
         * @param _abr
         * @param _label
         * @param _relationsData
         * @return
         * @throws SubarchitectureProcessException
         * @throws BindingComponentException
         */
    public static CASTData<BindingProxy> getLabeledRelation(
	    AbstractBindingReader _abr, String _label,
	    ArrayList<CASTData<BindingProxy>> _relationsData)
	    throws BindingComponentException, SubarchitectureProcessException {
	for (CASTData<BindingProxy> proxyData : _relationsData) {
	    BindingProxy proxy = proxyData.getData();
	    RelationLabel rl = getBindingFeature(_abr, _abr.getBindingSA(),
		    proxy, RelationLabel.class);
	    if (rl != null) {
		if (rl.m_label.equals(_label)) {
		    return proxyData;
		}
	    }
	}
	return null;
    }

    /**
         * @param _generator
         * @param _desiredPos
         * @param name
         * @return
         * @throws SubarchitectureProcessException
         * @throws BindingComponentException
         */
    public static <T> ArrayList<T> getBindingFeatures(
	    AbstractBindingReader _abr, BindingProxy _union, Class<T> _cls)
	    throws BindingComponentException, SubarchitectureProcessException {
	ArrayList<String> ids = findFeatures(_union, CASTUtils.typeName(_cls));
	ArrayList<T> features = new ArrayList<T>();
	for (String string : ids) {
	    features.add(getObject(_abr, string, _abr.getBindingSA(), _cls)
		    .getData());
	}
	return features;
    }

    /**
         * @param _abr
         * @param _bindingSA
         * @param _label
         * @param _locationUnion
         * @throws SubarchitectureProcessException
         * @throws BindingComponentException
         */
    public static BindingProxy getLabeledRelation(
	    WorkingMemoryReaderProcess _abr, String _bindingSA, String _label,
	    BindingUnion _locationUnion) throws BindingComponentException,
	    SubarchitectureProcessException {
	return getLabeledRelation(_abr, _bindingSA, _label, getRelations(_abr,
		_bindingSA, _locationUnion));
    }

    /**
         * @param _abr
         * @param _bindingSA
         * @param _proxy
         * @return
         * @throws SubarchitectureProcessException
         */
    public static BindingUnion getUnion(WorkingMemoryReaderProcess _abr,
	    String _bindingSA, BindingProxy _proxy)
	    throws SubarchitectureProcessException {
	return getUnion(_abr, _bindingSA, _proxy.m_unionID);
    }

    /**
         * @param _abr
         * @param _label
         * @param _proxy
         * @throws SubarchitectureProcessException
         * @throws BindingComponentException
         */
    public static ArrayList<BindingProxy> getLabeledRelations(
	    AbstractBindingReader _abr, String _label, BindingProxy _proxy)
	    throws BindingComponentException, SubarchitectureProcessException {
	return getLabeledRelations(_abr, _abr.getBindingSA(), _label, _proxy);
    }

    public static ArrayList<BindingProxy> getLabeledRelations(
	    WorkingMemoryReaderProcess _abr, String _bindingSA, String _label,
	    BindingProxy _proxy) throws BindingComponentException,
	    SubarchitectureProcessException {
	ArrayList<BindingProxy> rels = new ArrayList<BindingProxy>();

	ArrayList<BindingProxy> relations = getRelations(_abr, _bindingSA,
		_proxy);
	for (BindingProxy proxy : relations) {
	    RelationLabel rl = getBindingFeature(_abr, _bindingSA, proxy,
		    RelationLabel.class);
	    if (rl != null) {
		if (rl.m_label.equals(_label)) {
		    rels.add(proxy);
		}
	    }
	}
	return rels;
    }

    public static ArrayList<BindingProxy> getLabeledRelations(
	    WorkingMemoryReaderProcess _abr, String _bindingSA, String _label,
	    BindingUnion _union) throws BindingComponentException,
	    SubarchitectureProcessException {
	ArrayList<BindingProxy> rels = new ArrayList<BindingProxy>();

	ArrayList<BindingProxy> relations = getRelations(_abr, _bindingSA,
		_union);
	for (BindingProxy proxy : relations) {
	    RelationLabel rl = getBindingFeature(_abr, _bindingSA, proxy,
		    RelationLabel.class);
	    if (rl != null) {
		if (rl.m_label.equals(_label)) {
		    rels.add(proxy);
		}
	    }
	}
	return rels;
    }

    /**
         * @param _generator
         * @param _string
         * @param _proxy
         * @return
     * @throws SubarchitectureProcessException 
         */
    public static CASTData<BindingProxy> getLabeledRelationData(
	    AbstractBindingReader _abr, String _label, BindingProxy _proxy) throws SubarchitectureProcessException {

	ArrayList<CASTData<BindingProxy>> relations = getRelationsData(_abr,
		_proxy);
	
	for (CASTData<BindingProxy> proxy : relations) {
	    RelationLabel rl = getBindingFeature(_abr, proxy.getData(),
		    RelationLabel.class);
	    if (rl != null) {
		if (rl.m_label.equals(_label)) {
		    return proxy;
		}
	    }
	}
	return null;
    }


    // /**
    // * @param _abr
    // * @param _bindingSA
    // * @param _label
    // * @param _relations
    // * @return
    // */
    // private static Object getLabeledRelation(WorkingMemoryReaderProcess
    // _abr, String _bindingSA, String _label, ArrayList<BindingProxy>
    // _relations) {
    // // TODO Auto-generated method stub
    // return null;
    // }

}
