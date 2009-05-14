package coma.comparators;

import java.util.Properties;

import org.cognitivesystems.reasoner.base.OntologyMemberFactory;

import coma.aux.ComaFunctionWriter;

import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import binding.BindingException;
import binding.abstr.AbstractFeatureComparator;
import BindingData.*;
import BindingFeatures.AreaID;
import BindingFeatures.Concept;



public class AreaVsConceptComparisonReader  extends AbstractFeatureComparator {

	private ComaFunctionWriter m_funcWriter;
	private String m_ontologyNamespace;
	private String m_ontologySep;
	private OntologyMemberFactory m_oeMemberMaker;


	public AreaVsConceptComparisonReader (String _id) {
		super(_id);
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		m_behavior = Behavior.READ;
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
    	
		this.m_oeMemberMaker = new OntologyMemberFactory(m_ontologyNamespace, m_ontologySep);
		m_funcWriter = new ComaFunctionWriter(this);
	}	
	
	protected void startComparator() {
		log("startComparator");
		ComparisonTrustSpecification comparisonTrustSpecification = new ComparisonTrustSpecification();
		comparisonTrustSpecification.m_trustInternalFalse = ComparisonTrust.DONT_USE;
		comparisonTrustSpecification.m_trustInternalIndeterminate = ComparisonTrust.DONT_USE;
		comparisonTrustSpecification.m_trustInternalTrue = ComparisonTrust.DONT_USE;

		addFeatureComparisonFilter(CASTUtils.typeName(Concept.class), CASTUtils.typeName(AreaID.class), comparisonTrustSpecification);
		addFeatureComparisonFilter(CASTUtils.typeName(AreaID.class), CASTUtils.typeName(Concept.class), comparisonTrustSpecification);
		log("exiting startComparator");
	}


	
	@Override
	protected void dispatchComparison(String _comparisonID) {
		log("called dispatchComparison(_comparisonID:"+_comparisonID+")");

		// prepare the general info needed across functions
		AreaID _area = null;;
		Concept _concept = null;
		try {
			CASTData<?> _proxyFeature = getWorkingMemoryEntry(
					currentComparison().m_proxyFeature.m_address,
	        		currentComparison().m_bindingSubarchitectureID);
			
			CASTData<?> _unionFeature = getWorkingMemoryEntry(
					currentComparison().m_unionFeature.m_address,
	        		currentComparison().m_bindingSubarchitectureID);
			
	        if (_proxyFeature.getType().equals(
	        				CASTUtils.typeName(AreaID.class))) {
	        	_area = (AreaID) _proxyFeature.getData();
	        } else if (_proxyFeature.getType().equals(
    				CASTUtils.typeName(Concept.class))) {
	        	_concept = (Concept) _proxyFeature.getData();
	        } else {
	        	throw new RuntimeException("Got an illegal (proxy) feature type: " + _proxyFeature.getType() +
	        			" -- aborting!");
	        }
	    		
	        if (_unionFeature.getType().equals(
	        		CASTUtils.typeName(AreaID.class))) {
	        	_area = (AreaID) _unionFeature.getData();
	        } else if (_unionFeature.getType().equals(
	        		CASTUtils.typeName(Concept.class))) {
	        	_concept = (Concept) _unionFeature.getData();
	        } else {
	        	throw new RuntimeException("Got an illegal (union) feature type: " + _proxyFeature.getType() +
	        	" -- aborting!");
	        }
	    		
		} catch (BindingException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}

		log("My task is to compare AreaID:" + _area.m_id+ " vs. Concept:" + _concept.m_concept);
		if (_area==null || _concept.m_concept.equals("")) {
			log ("Warning: at least one feature is EMPTY! not performing this comparison! exiting...");
			return;
		}
		else {
			log ("This task is ok. Going to evaluate it...");
		}

		try {
			m_funcWriter.compareInstanceConcept(null, 
					m_oeMemberMaker.createInstance("area"+_area.m_id), 
					m_oeMemberMaker.createConcept(_concept.m_concept),
					new WorkingMemoryPointer(CASTUtils.typeName(FeatureComparison.class),
							new WorkingMemoryAddress(_comparisonID, currentComparison().m_bindingSubarchitectureID)));
		} catch (BindingException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
		log("added ComaReasonerFunction to WM. Exiting dispatchComparison()...");
	}



	@Override
	protected TriBool executeComparison() {
		// TODO Auto-generated method stub
		return null;
	}

}
