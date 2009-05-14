package coma.comparators;

import java.util.TreeMap;
import java.util.Properties;

import org.cognitivesystems.reasoner.base.OntologyMemberFactory;

import ComaData.ComaConcept;
import ComaData.ComaConceptComparisonResult;
import coma.aux.ComaFunctionWriter;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import binding.BindingException;
import binding.abstr.AbstractFeatureComparator;
import BindingData.*;
import BindingFeatures.Concept;


public class ConceptVsConceptBindingFeatureComparisonReader extends
AbstractFeatureComparator {
	
	private String m_ontologyNamespace;
	private String m_ontologySep;

	
	private class ConPair implements Comparable{
		private String m_concept1;
		private String m_concept2;
		
		public ConPair(String _concept1, String _concept2) {
			m_concept1 = _concept1;
			m_concept2 = _concept2;
		}

		public String getConcept1() {
			return m_concept1;
		}
		public String getConcept2() {
			return m_concept2;
		}

		public boolean equals(ConPair _otherConPair) {
			return (m_concept1.equals(_otherConPair.m_concept1) &&
					m_concept2.equals(_otherConPair.m_concept2)); 
		}

		public int compareTo(ConPair _otherConPair) {
			return (m_concept1+m_concept2).compareTo(_otherConPair.m_concept1+_otherConPair.m_concept2);
		}

		public int compareTo(Object o) {
			return (m_concept1+m_concept2).compareTo(((ConPair)o).m_concept1+((ConPair)o).m_concept2);
		}
		
		public String toString() {
			return "ConPair:(" + m_concept1 + " | " + m_concept2 +")";
		}
	}
	
	// the first field is the first concept
	private TreeMap<ConPair, TriBool> m_evaluationCache;
	private ComaFunctionWriter m_funcWriter;
	
	public ConceptVsConceptBindingFeatureComparisonReader(String _id) {
		super(_id);
		// TODO Auto-generated constructor stub

        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		m_behavior = Behavior.READ;
		m_evaluationCache = new TreeMap<ConPair, TriBool>();
	}

    public void configure(Properties _config) {
    	super.configure(_config);
        if (_config.containsKey("--onto_ns")) {
            this.m_ontologyNamespace = _config.getProperty("--onto_ns");
        }
        else {
//            this.m_ontologyNamespace = "http://www.dfki.de/cosy/officeenv.owl";
            this.m_ontologyNamespace = "oe";
        }
        if (_config.containsKey("--onto_sep")) {
            this.m_ontologySep= _config.getProperty("--onto_sep");
        }
        else {
//            this.m_ontologySep = "#";
            this.m_ontologySep = ":";
        }
		m_funcWriter = new ComaFunctionWriter(this);
    }	

	protected void startComparator() {
	    log("startComparator");
	    ComparisonTrustSpecification comparisonTrustSpecification = new ComparisonTrustSpecification();
	    comparisonTrustSpecification.m_trustInternalFalse = ComparisonTrust.DONT_USE;
	    comparisonTrustSpecification.m_trustInternalIndeterminate = ComparisonTrust.DONT_USE;
	    comparisonTrustSpecification.m_trustInternalTrue = ComparisonTrust.TRUST_COMPLETELY; // since a string comparison is trivial
	    
	    addFeatureComparisonFilter(CASTUtils.typeName(Concept.class),comparisonTrustSpecification);
		
		try {
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					ComaConceptComparisonResult.class, WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			            	log("I got an UPDATE for " + CASTUtils.toString(_wmc));
			                processUpdate(_wmc);
						}
			});
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	    log("exiting startComparator");
	}

	protected void dispatchComparison(String _comparisonID) {
		log("called dispatchComparison(_comparisonID:"+_comparisonID+")");
		log("The evaluation cache has size: " + m_evaluationCache.size());
		log(m_evaluationCache);

		// in order to compare two concepts
			// I need to know whether they are
			// equivalent (-> TriTrue)
			// or c1 is a subconcept of c2 (-> TriTrue)
			// or c1 is a superconcept of c2 (-> TriIndeterminate).
			// Otherwise -> TriFalse;
			
			// prepare the general info needed across functions
			Concept _con1;
			try {
				_con1 = (Concept) getWorkingMemoryEntry(currentComparison().
						m_proxyFeature.m_address,currentComparison().m_bindingSubarchitectureID).getData();
			} catch (BindingException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
			Concept _con2;
			try {
				_con2 = (Concept) getWorkingMemoryEntry(currentComparison().
						m_unionFeature.m_address,currentComparison().m_bindingSubarchitectureID).getData();
			} catch (BindingException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
			
			log("My task is to compare " + _con1.m_concept + " vs. " + _con2.m_concept);
			if ((_con1.m_concept.equals("")) || (_con2.m_concept.equals(""))) {
				log ("Warning: one concept string is EMPTY! not performing this comparison! exiting...");
				return;
			}
			else if ((_con1.m_concept == null) || (_con2.m_concept==null)) {
				log ("Warning: one concept string is NULL! not performing this comparison! exiting...");
				return;
			}
			else {
				log ("This task is ok. Going to evaluate it...");
			}
			
			// this is a "hack" to connect to ComSys.mk4!
			//log("HACK! Splitting concept names along the colon...");
			//if (_con1.m_concept.contains(":")) _con1.m_concept=_con1.m_concept.split(":")[1];
			//if (_con2.m_concept.contains(":")) _con2.m_concept=_con2.m_concept.split(":")[1];
			
			// identical names will always yield "TriTrue"!
			if (_con1.m_concept.equals(_con2.m_concept)) {
				log("identical concept names...");
				try {
					updateScoreWME(new WorkingMemoryAddress(_comparisonID,currentComparison().m_bindingSubarchitectureID) , TriBool.triTrue);
				} catch (BindingException e) {
					e.printStackTrace();
					throw new RuntimeException(e);
				}
				log("exiting dispatchComparison()...");
				return;
			}
			
			// OK, now check the cache whether we actually need to carry out
			// the evaluation via the COMA
			// or whether we have a cached result that we can use instead
			if (m_evaluationCache.containsKey(new ConPair(_con1.m_concept, _con2.m_concept))) {
				log("cache contains the key...");
				if (m_evaluationCache.get(new ConPair(_con1.m_concept, _con2.m_concept))!=null) {
					log ("cache entry is not null...");
					TriBool _comparisonResult = m_evaluationCache.get(new ConPair(_con1.m_concept, _con2.m_concept));
					log("my current comparison's original value is: " + triBool2String(m_currentComparison.originalValue));
					log("going to update score WMW for ID " + _comparisonID + " with result " + triBool2String(_comparisonResult));
					try {
						updateScoreWME(new WorkingMemoryAddress(_comparisonID, currentComparison().m_bindingSubarchitectureID), _comparisonResult);
					} catch (BindingException e) {
						e.printStackTrace();
						throw new RuntimeException(e);
					}
					log("USE CACHED COMPARISON... no need to query the COMA reasoner!");
					log("COMPARISON: " + _con1.m_concept + " vs. " 
							+ _con2.m_concept + ": " + triBool2String(_comparisonResult));
					log("exiting dispatchComparison()...");
					return;
				}
			}
			
			log("The current comparison has not yet been cached, so I am going to query the reasoner...");
			// we should only be here for non-cached comparisons
			// so it is a good idea to start caching the current comparison
			m_evaluationCache.put(new ConPair(_con1.m_concept, _con2.m_concept), null);
			
			// new outsourced code, 2008-06-03, hz
			try {
				m_funcWriter.compareCons(null, 
						OntologyMemberFactory.createConcept("", "", _con1.m_concept), 
						OntologyMemberFactory.createConcept("", "", _con2.m_concept),
						new WorkingMemoryPointer(CASTUtils.typeName(FeatureComparison.class),
								new WorkingMemoryAddress(_comparisonID, currentComparison().m_bindingSubarchitectureID)));
			} catch (BindingException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
			
//			String _c1ID = newDataID();
//			String _c2ID = newDataID();
//
//			addToWorkingMemory(_c1ID, new ComaConcept("","",_con1.m_concept), OperationMode.BLOCKING);
//			addToWorkingMemory(_c2ID, new ComaConcept("","",_con2.m_concept), OperationMode.BLOCKING);
//
//			WorkingMemoryPointer _arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
//					new WorkingMemoryAddress(_c1ID, m_subarchitectureID));
//			WorkingMemoryPointer _arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
//					new WorkingMemoryAddress(_c2ID, m_subarchitectureID));
//			
//			// NEW
//			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.CompareCons,_arg1ptr,
//					_arg2ptr,new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
//					 new WorkingMemoryPointer(
//								CASTUtils.typeName(FeatureComparison.class),
//								new WorkingMemoryAddress(_comparisonID, currentComparison().m_bindingSubarchitectureID)));
//			String _reqID = newDataID();
//			addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);

			log("added ComaReasonerFunction to WM. Exiting dispatchComparison()...");
			
			// OLD
			/*
			// now prepare the 3 requests (aka functions)
			ComaReasonerFunction _req1 = new ComaReasonerFunction();
			ComaReasonerFunction _req2 = new ComaReasonerFunction();
			ComaReasonerFunction _req3 = new ComaReasonerFunction();
			_req1.m_functiontype = ComaReasonerFunctionType.AreConsEquivalent;
			_req2.m_functiontype = ComaReasonerFunctionType.IsConSubcon;
			_req3.m_functiontype = ComaReasonerFunctionType.IsConSupercon;

			_req1.m_arg1ptr = _arg1ptr;
			_req2.m_arg1ptr = _arg1ptr;
			_req3.m_arg1ptr = _arg1ptr;
			_req1.m_arg2ptr = _arg2ptr;
			_req2.m_arg2ptr = _arg2ptr;
			_req3.m_arg2ptr = _arg2ptr;

			String _req1ID = newDataID();
			String _req2ID = newDataID();
			String _req3ID = newDataID();
			
			// now add the task to the WM that will trigger the WRITE 
			// counterpart of this to collect the answers and
			// write back to binding WM
			// (we do it before we dispatch the actual functions
			//  to give the WRITE component a headstart)
			ComaTask _myTask = new ComaTask();
			_myTask.m_tasktype = ComaTaskType.ConceptComparisonTask;
			_myTask.m_function_ptr_list = new WorkingMemoryPointer[3];
			_myTask.m_function_ptr_list[0] = new WorkingMemoryPointer(
					ComaOntology.COMA_REASONER_FUNCTION_TYPE, 
					new WorkingMemoryAddress(_req1ID,m_subarchitectureID));
			_myTask.m_function_ptr_list[1] = new WorkingMemoryPointer(
					ComaOntology.COMA_REASONER_FUNCTION_TYPE, 
					new WorkingMemoryAddress(_req2ID,m_subarchitectureID));
			_myTask.m_function_ptr_list[2] = new WorkingMemoryPointer(
					ComaOntology.COMA_REASONER_FUNCTION_TYPE, 
					new WorkingMemoryAddress(_req3ID,m_subarchitectureID));
			_myTask.m_add_info_ptr = new WorkingMemoryPointer(
					BindingOntology.FEATURE_COMPARISON_TYPE,
					new WorkingMemoryAddress(_comparisonID, m_bindingSA));
			
			log("As far as I know, the binding WME's address (id) is: " + _comparisonID);
			
			String _myTaskID = newDataID();
			addToWorkingMemory(_myTaskID, ComaOntology.COMA_TASK_TYPE, _myTask);
			
			// now write the functions to WM
			addToWorkingMemory(_req1ID, ComaOntology.COMA_REASONER_FUNCTION_TYPE, _req1);
			addToWorkingMemory(_req2ID, ComaOntology.COMA_REASONER_FUNCTION_TYPE, _req2);
			addToWorkingMemory(_req3ID, ComaOntology.COMA_REASONER_FUNCTION_TYPE, _req3);
			*/			// We're done now!
//		} catch (SubarchitectureProcessException e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//		}
	}

	public void processUpdate(WorkingMemoryChange _wmc) {
		log("called processUpdate(_wmc"+_wmc+")");
			CASTData<?> wme;
			try {
				wme = getWorkingMemoryEntry(_wmc.m_address);
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException(e);
			}
			//FeatureComparisonTask task = (FeatureComparisonTask) wme.getData();
			//FeatureComparison comparison = (FeatureComparison) getWorkingMemoryEntry(task.m_comparisonID,m_bindingSA).getData();
 			ComaConceptComparisonResult comparison = (ComaConceptComparisonResult) wme.getData();

			ComaConcept _con1 = comparison.m_con1;
			ComaConcept _con2 = comparison.m_con2;
			
            // update internal cache!
            TriBool _res = comparison.m_result;
            
            ConPair _cons = new ConPair(_con1.m_name, _con2.m_name);
            m_evaluationCache.put(_cons, _res);
            log("Updating concept comparison cache: " + _cons.m_concept1 + " vs. " + _cons.m_concept2 + " => " + triBool2String(_res));
        log("exiting processUpdate()");
	}


	@Override
	protected TriBool executeComparison() {
		// TODO Auto-generated method stub
		return null;
	}

}
