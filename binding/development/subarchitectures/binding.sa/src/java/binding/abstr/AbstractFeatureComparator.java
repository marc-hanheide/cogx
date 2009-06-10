package binding.abstr;

import java.util.TreeMap;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.TreeSet;

import binding.BindingException;
import BindingData.BINDING_SUBARCH_CONFIG_KEY;
import BindingData.FeatureComparison;
import BindingData.ComparisonTrust;
import BindingData.ComparisonTrustSpecification;
import BindingData.FeatureComparisonCompetence;
import BindingData.FeatureComparisonTask;
import BindingFeatures.Concept;
import binding.common.BindingComponentException;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.TriBool;
import cast.cdl.FilterRestriction;
import cast.cdl.OperationMode;

import cast.core.data.CASTData;

/**
 * inherit from this class. The class can expose three different
 * behaviors (you need to initialize and set that flag in your
 * constructor!): READ only WRITE only READWRITE The simplest case is
 * READWRITE: your class gets a feature comparison task, evaluates it
 * and writes the result back. In case, the feature comparison needs to
 * be handled by other components of your subarchitecture, you need two
 * components for handling the feature comparison: a READer and a
 * WRITEer component. The READ component gets a feature comparison
 * request and dispatches the evaluation. You need to implement the way
 * you request information from other components *and* you need to make
 * sure the id of the original feature comparison task is handed over...
 * ...to the WRITE component. This component will write the comparison
 * result back to the Binding WM. You need to make sure that this class
 * collects the results of your SA-internal requests *and* knows where
 * to write the result back to. I suggest you pass the WME-id of the
 * original feature comparison task on together with the information
 * requests and your WRITE components just waits for UPDATEs of the
 * request structs. But this you have to implement yourself, esp. incl.
 * IDL!
 * 
 * @author Hendrik Zender, Henrik Jacobsson, {zender|henrikj}@dfki.de
 */
public abstract class AbstractFeatureComparator
        extends
            PrivilegedManagedProcess {

    // You must specify the behavior of your inheriting class
    // READWRITE means that one and the same component will
    // get a feature comparison task, work it out,
    // and write the result back to the task struct.
    // READ and WRITE analogous...
    public enum Behavior {
        READ, WRITE, READWRITE
    };

    protected Behavior m_behavior;

    protected Comparison m_currentComparison;

    // maps from proxy feature type string to union feature type string
    private Map<String, Map<String, ComparisonTrustSpecification>> m_filter;

    // the ID of the binding subarchitecture
    protected String Xm_bindingSA; // ???

    // / the IDs of all binding SAs on which the competence is
    // registered
    protected Set<String> m_bindingSA;

    /**
     * If you inherit from this abstract class you must set the behavior
     * flag in the constructor!
     * 
     * @param _id
     */
    public AbstractFeatureComparator(String _id) {
        super(_id);
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
        m_filter =
                new TreeMap<String, Map<String, ComparisonTrustSpecification>>();
        m_currentComparison = new Comparison();
        m_bindingSA = new TreeSet();
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        if (m_behavior == null) {
            throw new RuntimeException("ERROR: NO BEHAVIOR SPECIFIED!");
        }

        super.start();

        if (m_behavior!=Behavior.WRITE) { // to prevent too many
        	// registrations...
        	try {
        		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(FeatureComparisonTask.class, WorkingMemoryOperation.ADD),
        				new WorkingMemoryChangeReceiver() {

        			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
        				// log(CASTUtils.toString(_wmc));
        				_processFeatureComparisonTask(_wmc);
        			}
        		});
        	}
        	catch (SubarchitectureProcessException e) {
        		e.printStackTrace();
        		throw new RuntimeException("a SubarchitectureProcessException was thrown, aborting...");
        	}
        }

        // nah: moved to runComponent... you can't write before then
        // startComparator();
    }

    /**
     * is called from this->start() and must be overloaded with the
     * relevant calls to
     * 
     * @see #addFeatureComparisonFilter(String)
     */
    protected abstract void startComparator();

    // / If you overload configure, don't forget to call
    // / AbstractFeatureComparator::configure(...) too! (Maybe this
    // / should be made safer, as with start()...)
    public void configure(Properties _config) {
        super.configure(_config);
        if (m_behavior != Behavior.WRITE) {
            if (_config.containsKey(BINDING_SUBARCH_CONFIG_KEY.value)) {

                String ids =
                        _config
                            .getProperty(BINDING_SUBARCH_CONFIG_KEY.value);
                String[] subarchs = ids.split(",");
                for (String subarch : subarchs) {
                    m_bindingSA.add(subarch);
                    log("adding binding subarch: " + subarch);
                }
            }
            else {
                throw new RuntimeException(
                    "binding subarchs not specified! All of them must be sepcified to register the comparator competence");
            }
        }
    }

    /**
     * This method adds the pair of features to the ones that should be
     * compared by the comparator. The types can be different (if the
     * comparator is nonreflexive), but are typically not. Use this
     * function in the startComparator method. It is only called for
     * READ and READWRITE comparators.
     * 
     * @param _proxyFeatureType
     * @param _unionFeatureType
     */
    protected void addFeatureComparisonFilter(String _proxyFeatureType,
                                              String _unionFeatureType,
                                              ComparisonTrustSpecification _comparisonTrustSpecification) {
        Map<String, ComparisonTrustSpecification> _value =
                m_filter.get(_proxyFeatureType);
        if (_value == null) {
            _value =
                    new TreeMap<String, ComparisonTrustSpecification>();
        }
        _value.put(_unionFeatureType, _comparisonTrustSpecification);
        m_filter.put(_proxyFeatureType, _value);
        for (String subarch : m_bindingSA) {
            FeatureComparisonCompetence competence =
                    new FeatureComparisonCompetence();
            competence.m_proxyFeatureType = _proxyFeatureType;
            competence.m_unionFeatureType = _unionFeatureType;
            competence.m_comparisonTrustSpecification =
                    _comparisonTrustSpecification;
            log("Registring competence on: " + subarch);
            try {
                addToWorkingMemory(newDataID(), 
				   subarch,
				   competence,
				   OperationMode.BLOCKING);
            }
            catch (SubarchitectureProcessException _e) {
                throw new RuntimeException("Not a subarch perhaps: "
                    + _e);
            }
        }
    }

    /**
     * calls addFeatureComparisonFilter(_featureType,_featureType)...
     * Only relevant for READ and READWRITE comparators...
     * 
     * @param _featureType
     */
    protected void addFeatureComparisonFilter(String _featureType,
                                              ComparisonTrustSpecification _comparisonTrustSpecification) {
        addFeatureComparisonFilter(_featureType, _featureType,
            _comparisonTrustSpecification);
    }

    /**
     * READWRITE: Overload this with your code. This is supposed to
     * return true, false or indeterminate based on whatever algorithm
     * you are using to do this. You will need to load the feature data
     * yourself. Access the featureComparison via currentComparison().
     * This method returning a TriBool is called for READWRITE
     * comparators only!
     */
    protected abstract TriBool executeComparison();

    /**
     * READ: Overload this with your code. This is supposed to return
     * true, false or indeterminate based on whatever algorithm you are
     * using to do this. You will need to load the feature data
     * yourself. Access the featureComparison via currentComparison().
     * This method returning void is called for READ comparators only!
     */
    protected abstract void dispatchComparison(String _comparisonID);

    /**
     * READ/READWRITE: This code is called as soon as a comparison task
     * is added to WM. This loads the feature comparison struct and
     * checks whether the given comparison is supposed to be handled by
     * this specific comparator. If the task is relevant, it is deleted
     * from WM, and the feature comparison evaluation is triggered.
     * Depending on the behavior of your inherited class, either
     * 
     * @see #executeComparison() (for READWRITE) or
     * @see #dispatchComparison() (for READ) is called. The READWRITE
     *      version immediatley gets the comparison result; if the
     *      result differs from the existing one it will be overwritten.
     * @param _wmc
     */
    private void _processFeatureComparisonTask(WorkingMemoryChange _wmc) {
        // WRITE comparators are not supposed to call this method!
        if (m_behavior == Behavior.WRITE) {
        	log("_processFeatureComparisonTask(_wmc.m_address.m_id:"+_wmc.m_address.m_id+") called! WRITE comparators should not call this method!...exiting");
        	throw new RuntimeException("_processFeatureComparisonTask(_wmc.m_address.m_id:"+_wmc.m_address.m_id+") called! WRITE comparators should not call this method!...exiting");
//        	return;
        }
        try {
            CASTData<?> wme;
            try {
                wme = getWorkingMemoryEntry(_wmc.m_address);
            }
            catch (DoesNotExistOnWMException _e) {
                log("Task at: " + _wmc.m_address.m_id
                    + " does not exist");
                return;
            }
            FeatureComparisonTask task =
                    (FeatureComparisonTask) wme.getData();

            FeatureComparison comparison =
                    (FeatureComparison) getWorkingMemoryEntry(
                        task.m_comparisonID,
                        task.m_bindingSubarchitectureID).getData();

            log(comparison);

            m_currentComparison.featureComparison = comparison;

            log(currentComparison().m_proxyFeature.m_type + " vs. "
                + currentComparison().m_unionFeature.m_type);

            if (!currentComparisonIsMyTask()) {
                // i.e., not our task... do nothing
                log("not my task!"
                    + currentComparison().m_proxyFeature.m_type
                    + " vs. "
                    + currentComparison().m_unionFeature.m_type);
                m_currentComparison = new Comparison();
                return;
            }

            // this is our task! delete the task specification right
            // away
            deleteFromWorkingMemory(_wmc.m_address.m_id,
                _wmc.m_address.m_subarchitecture);

            m_currentComparison.id = task.m_comparisonID;
            log("currentComparisonID = " + m_currentComparison.id);
            m_currentComparison.originalValue =
                    comparison.m_featuresEquivalent;

            // FOR READWRITE COMPARATORS ONLY:
            if (m_behavior == Behavior.READWRITE) {
                m_currentComparison.newValue = executeComparison();

                log("comparison result: "
                    + triBool2String(m_currentComparison.newValue)
                    + " (old value: "
                    + triBool2String(m_currentComparison.originalValue)
                    + ")");

                // only store if resulting value is actually different
                if (m_currentComparison.featureComparison.m_insistOnExternalComparison
                    || m_currentComparison.originalValue != m_currentComparison.newValue) {
                    updateScoreWME(
                        new WorkingMemoryAddress(
                            m_currentComparison.id,
                            m_currentComparison.featureComparison.m_bindingSubarchitectureID),
                        m_currentComparison.newValue);
                }
            }
            // FOR READ COMPARATORS ONLY:
            else {
            	log("calling dispatchComparison("+m_currentComparison.id+")");
                dispatchComparison(m_currentComparison.id);
            }
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }
        m_currentComparison.featureComparison = new FeatureComparison();
        log("exiting _processFeatureComparison()");
    }

    protected void updateScoreWME(WorkingMemoryAddress _currentComparisonWMA,
                                  TriBool _result) {
        log("called updateScoreWME(_currentComparisonWMA:"+_currentComparisonWMA+" , _result:"+triBool2String(_result)+")");
    	try {
            CASTData<?> featCompWME =
                    getWorkingMemoryEntry(_currentComparisonWMA); // new
            // WorkingMemoryAddress(_currentComparisonID,
            // m_bindingSA));
            FeatureComparison featrComp =
                    (FeatureComparison) featCompWME.getData();

            // Concept _con1 =
            // (Concept) getWorkingMemoryEntry(featrComp.
            // m_proxyFeature.m_address,m_bindingSA).getData();
            // Concept _con2 =
            // (Concept) getWorkingMemoryEntry(featrComp.
            // m_unionFeature.m_address,m_bindingSA).getData();

            featrComp.m_featuresEquivalent = _result;

            // log("updateScoreWME("+_currentComparisonID+","+_con1.m_concept+"
            // vs. "+_con2.m_concept+" =>
            // "+triBool2String(_result)+")");
            overwriteWorkingMemory(_currentComparisonWMA.m_id, // _currentComparisonID,
				   _currentComparisonWMA.m_subarchitecture, // m_bindingSA,
				   featrComp,
				   OperationMode.BLOCKING);
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }
    }

    /**
     * This is a helper class for producing a String of a TriBool value.
     * 
     * @param _tribool
     * @return a String representation of the given TriBool
     */
    protected String triBool2String(TriBool _tribool) {
        switch (_tribool.value()) {
            case TriBool._triFalse:
                return "triFalse";
            case TriBool._triTrue:
                return "triTrue";
            case TriBool._triIndeterminate:
                return "triIndeterminate";
            default:
                break;
        }
        return "Error converting TriBool!";
    }

    /**
     * use this to get the feature comparison that you're supposed to
     * process
     * 
     * @return
     * @throws BindingException
     */
    protected FeatureComparison currentComparison()
            throws BindingException {
        if (m_currentComparison.featureComparison == null)
            throw new BindingException(
                "No current comparison loaded error.");
        return m_currentComparison.featureComparison;
    }

    // nested "struct"
    protected class Comparison {

        public FeatureComparison featureComparison;
        public String id;
        public TriBool originalValue;
        public TriBool newValue;
    }

    /**
     * returns true if the current comparison task is relevant for this
     * comparator (according to \p m_filter)
     * 
     * @return
     */
    protected boolean currentComparisonIsMyTask() {
        if (m_currentComparison.featureComparison == null) {
            debug("m_currentComparison.featureComparison == null -> current comparison is not my task");
        	return false;
        }
        try {
            String proxy_feature_type =
                    currentComparison().m_proxyFeature.m_type;
            if (!m_filter.containsKey(proxy_feature_type)) {
            	debug("!m_filter.containsKey(proxy_feature_type = "+proxy_feature_type+") -> current comparison is not my task");
            	debug(m_filter);
            	return false;
            }
            else {
                String union_feature_type =
                        currentComparison().m_unionFeature.m_type;
                if (!m_filter.get(proxy_feature_type).containsKey(
                    union_feature_type)) {
                	debug("!m_filter.get(proxy_feature_type = "+proxy_feature_type+").containsKey(union_feature_type ="+union_feature_type+") -> current comparison is not my task");
                	return false;
                }
            }
            debug("current comparison is my task!");
            return true;
        }
        catch (BindingException e) {
            log(e.getMessage() + e.getStackTrace());
            return false;
        }
    }

    @Override
    protected void taskAdopted(String _taskID) {
    // TODO Auto-generated method stub

    }

    @Override
    protected void taskRejected(String _taskID) {
    // TODO Auto-generated method stub

    }

    /**
     * Calls startComparator(). If you overide this class and uses
     * runComponent, make sure you call
     * {@link AbstractFeatureComparator}.runComponent() at the start of
     * your runComponent method.
     */
    @Override
    protected void runComponent() {
        startComparator();
    }

}
