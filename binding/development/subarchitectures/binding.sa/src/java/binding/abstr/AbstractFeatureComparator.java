package binding.abstr;

import java.util.TreeMap;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.TreeSet;

import binding.BindingException;
import BindingData.BINDINGSUBARCHCONFIGKEY;
import BindingData.FeatureComparison;
import BindingData.ComparisonTrust;
import BindingData.ComparisonTrustSpecification;
import BindingData.FeatureComparisonCompetence;
import BindingData.FeatureComparisonTask;
import BindingFeatures.Concept;
import binding.common.BindingComponentException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.DoesNotExistOnWMException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import BindingData.TriBool;
import cast.cdl.FilterRestriction;
// import cast.cdl.OperationMode;

import cast.core.CASTData;

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
            ManagedComponent {

    // You must specify the behavior of your inheriting class
    // READWRITE means that one and the same component will
    // get a feature comparison task, work it out,
    // and write the result back to the task struct.
    // READ and WRITE analogous...
    public enum Behavior {
        READ, WRITE, READWRITE
    };

    protected Behavior behavior;

    protected Comparison currentComparison;

    // maps from proxy feature type string to union feature type string
    private Map<String, Map<String, ComparisonTrustSpecification>> filter;

    // the ID of the binding subarchitecture
    protected String XbindingSA; // ???

    // / the IDs of all binding SAs on which the competence is
    // registered
    protected Set<String> bindingSA;

    /**
     * If you inherit from this abstract class you must set the behavior
     * flag in the constructor!
     * 
     * @param _id
     */
    public AbstractFeatureComparator(String _id) {
        super();
        WorkingMemoryChangeQueueBehaviour queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
        filter =
                new TreeMap<String, Map<String, ComparisonTrustSpecification>>();
        currentComparison = new Comparison();
        bindingSA = new TreeSet();
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderComponent#start()
     */
    @Override
    public void start() {
        if (behavior == null) {
            throw new RuntimeException("ERROR: NO BEHAVIOR SPECIFIED!");
        }

        super.start();

        if (behavior!=Behavior.WRITE) { // to prevent too many
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
        	catch (Exception e) {
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
    public void configure(Map<String,String> _config) {
        super.configure(_config);
        if (behavior != Behavior.WRITE) {
            if (_config.containsKey(BINDINGSUBARCHCONFIGKEY.value)) {

                String ids =
                        _config.get(BINDINGSUBARCHCONFIGKEY.value);
                String[] subarchs = ids.split(",");
                for (String subarch : subarchs) {
                    bindingSA.add(subarch);
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
                filter.get(_proxyFeatureType);
        if (_value == null) {
            _value =
                    new TreeMap<String, ComparisonTrustSpecification>();
        }
        _value.put(_unionFeatureType, _comparisonTrustSpecification);
        filter.put(_proxyFeatureType, _value);
        for (String subarch : bindingSA) {
            FeatureComparisonCompetence competence =
                    new FeatureComparisonCompetence();
            competence.proxyFeatureType = _proxyFeatureType;
            competence.unionFeatureType = _unionFeatureType;
            competence.comparisonTrustSpecs =
                    _comparisonTrustSpecification;
            log("Registring competence on: " + subarch);
            try {
                addToWorkingMemory(newDataID(), 
				   subarch,
				   competence);
            }
            catch (SubarchitectureComponentException _e) {
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
        if (behavior == Behavior.WRITE) {
        	log("_processFeatureComparisonTask(_wmc.address.id:"+_wmc.address.id+") called! WRITE comparators should not call this method!...exiting");
        	throw new RuntimeException("_processFeatureComparisonTask(_wmc.address.id:"+_wmc.address.id+") called! WRITE comparators should not call this method!...exiting");
//        	return;
        }
        try {
            CASTData<?> wme;
            try {
                wme = getWorkingMemoryEntry(_wmc.address);
            }
            catch (DoesNotExistOnWMException _e) {
                log("Task at: " + _wmc.address.id
                    + " does not exist");
                return;
            }
            FeatureComparisonTask task =
                    (FeatureComparisonTask) wme.getData();

            FeatureComparison comparison =
                    (FeatureComparison) getWorkingMemoryEntry(
                        task.comparisonID,
                        task.bindingSubarchitectureID).getData();

            log(comparison);

            currentComparison.featureComparison = comparison;

            log(currentComparison().proxyFeature.type + " vs. "
                + currentComparison().unionFeature.type);

            if (!currentComparisonIsMyTask()) {
                // i.e., not our task... do nothing
                log("not my task!"
                    + currentComparison().proxyFeature.type
                    + " vs. "
                    + currentComparison().unionFeature.type);
                currentComparison = new Comparison();
                return;
            }

            // this is our task! delete the task specification right
            // away
            deleteFromWorkingMemory(_wmc.address.id,
                _wmc.address.subarchitecture);

            currentComparison.id = task.comparisonID;
            log("currentComparisonID = " + currentComparison.id);
            currentComparison.originalValue =
                    comparison.featuresEquivalent;

            // FOR READWRITE COMPARATORS ONLY:
            if (behavior == Behavior.READWRITE) {
                currentComparison.newValue = executeComparison();

                log("comparison result: "
                    + triBool2String(currentComparison.newValue)
                    + " (old value: "
                    + triBool2String(currentComparison.originalValue)
                    + ")");

                // only store if resulting value is actually different
                if (currentComparison.featureComparison.insistOnExternalComparison
                    || currentComparison.originalValue != currentComparison.newValue) {
                    updateScoreWME(
                        new WorkingMemoryAddress(
                            currentComparison.id,
                            currentComparison.featureComparison.bindingSubarchitectureID),
                        currentComparison.newValue);
                }
            }
            // FOR READ COMPARATORS ONLY:
            else {
            	log("calling dispatchComparison("+currentComparison.id+")");
                dispatchComparison(currentComparison.id);
            }
        }
        catch (SubarchitectureComponentException e) {
            e.printStackTrace();
        }
        currentComparison.featureComparison = new FeatureComparison();
        log("exiting _processFeatureComparison()");
    }

    protected void updateScoreWME(WorkingMemoryAddress _currentComparisonWMA,
                                  TriBool _result) {
        log("called updateScoreWME(_currentComparisonWMA:"+_currentComparisonWMA+" , _result:"+triBool2String(_result)+")");
    	try {
            CASTData<?> featCompWME =
                    getWorkingMemoryEntry(_currentComparisonWMA); // new
            // WorkingMemoryAddress(_currentComparisonID,
            // bindingSA));
            FeatureComparison featrComp =
                    (FeatureComparison) featCompWME.getData();

            // Concept _con1 =
            // (Concept) getWorkingMemoryEntry(featrComp.
            // proxyFeature.address,bindingSA).getData();
            // Concept _con2 =
            // (Concept) getWorkingMemoryEntry(featrComp.
            // unionFeature.address,bindingSA).getData();

            featrComp.featuresEquivalent = _result;

            // log("updateScoreWME("+_currentComparisonID+","+_con1.concept+"
            // vs. "+_con2.concept+" =>
            // "+triBool2String(_result)+")");
            overwriteWorkingMemory(_currentComparisonWMA.id, // _currentComparisonID,
				   _currentComparisonWMA.subarchitecture, // bindingSA,
				   featrComp);
        }
        catch (SubarchitectureComponentException e) {
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
            case TriBool._FALSETB:
                return "triFalse";
            case TriBool._TRUETB:
                return "triTrue";
            case TriBool._INDETERMINATETB:
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
        if (currentComparison.featureComparison == null)
            throw new BindingException(
                "No current comparison loaded error.");
        return currentComparison.featureComparison;
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
     * comparator (according to \p filter)
     * 
     * @return
     */
    protected boolean currentComparisonIsMyTask() {
        if (currentComparison.featureComparison == null) {
            debug("currentComparison.featureComparison == null -> current comparison is not my task");
        	return false;
        }
        try {
            String proxy_feature_type =
                    currentComparison().proxyFeature.type;
            if (!filter.containsKey(proxy_feature_type)) {
            	debug("!filter.containsKey(proxy_feature_type = "+proxy_feature_type+") -> current comparison is not my task");
            	debug(filter);
            	return false;
            }
            else {
                String union_feature_type =
                        currentComparison().unionFeature.type;
                if (!filter.get(proxy_feature_type).containsKey(
                    union_feature_type)) {
                	debug("!filter.get(proxy_feature_type = "+proxy_feature_type+").containsKey(union_feature_type ="+union_feature_type+") -> current comparison is not my task");
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
