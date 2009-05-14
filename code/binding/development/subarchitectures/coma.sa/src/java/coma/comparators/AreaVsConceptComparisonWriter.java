package coma.comparators;


import java.util.Properties;

import coma.aux.ComaFunctionWriter;

import ComaData.ComaReasonerFunction;
import ComaData.ComaReasonerFunctionType;
import ComaData.TriBoolResult;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import BindingData.FeatureComparison;
import binding.abstr.AbstractFeatureComparator;

public class AreaVsConceptComparisonWriter  extends AbstractFeatureComparator {

	private ComaFunctionWriter m_funcWriter;
	private String m_ontologyNamespace;
	private String m_ontologySep;
	
	public AreaVsConceptComparisonWriter(String _id) {
		super(_id);
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		m_behavior = Behavior.WRITE;
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
		try {
			addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ComaReasonerFunction.class,
					WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					log("Got a callback: OVERWRITE on a COMA_REASONER_FUNCTION_TYPE!");
					debug(CASTUtils.toString(_wmc));
					processFunctionEvaluated(_wmc);
				}
			});
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException("Error! Failed to register change filter. Aborting!");
		}
	}


	/**
	 * This method is triggered when a COMA_REASONER_FUNCTION_TYPE WME
	 * has been OVERWRITTEN.
	 */
	private void processFunctionEvaluated(WorkingMemoryChange _wmc) {
		log("Got an UPDATE for a COMA REASONER FUNCTION: " + _wmc + 
				" from " +_wmc.m_address.m_subarchitecture);
		// foreplay: get the changed WME!
		CASTData<?> wme = null;
		ComaReasonerFunction _comaRsnrFn;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			log("The WME does not exist anymore. Someone else has deleted it, so I assume it was not meant for me anyway... returning.");
			return;
		}
		_comaRsnrFn= (ComaReasonerFunction) wme.getData();

		boolean _funcTypeIsFeatComp = _comaRsnrFn.m_functiontype.equals(ComaReasonerFunctionType.IsInstanceOf);
		if (!(_funcTypeIsFeatComp)) {
			log("This is not a IsInstanceOf ComaTask, so it cannot be relevant for me. returning...");
			return;
		}

		boolean _funcHasAddInfo = _comaRsnrFn.m_add_info_ptr.m_address.m_id.length()!=0;
		boolean _addInfoWMEExists = false;

		if (_funcHasAddInfo) {
			try {
				_addInfoWMEExists = existsOnWorkingMemory(_comaRsnrFn.m_add_info_ptr.m_address);
			} catch (SubarchitectureProcessException e) {
				log("Function has add info pointer, but it does not lead anywhere... I assume the function is not relevant for me. Returning...");
				return;
			}
		} else {
			log("Function does not have add info, so it cannot be relevant for me. returning...");
			return;
		}

		if (_addInfoWMEExists) {
			try {
				if (getWorkingMemoryEntry(_comaRsnrFn.m_add_info_ptr.m_address).getType().
						equals(CASTUtils.typeName(FeatureComparison.class))) {
					log("This function had been initiated because of a feature comparison request from the binder! So it's relevant for me!");
				} else {
					log("This function had not been initiated because of a feature comparison request from the binder, so it cannot be relevant. returning...");
					return;
				}
			} catch (SubarchitectureProcessException e) {
				log("WME does not exist anymore, so it cannot have been relevant for me. returning...");
				return;
			}
		} else {
			log("This function had not been initiated because of a feature comparison request from the binder, so it cannot be relevant. returning...");
			return;
		}
		log("This WME is an areaID vs concept comparison! trying to get the result WME...");

		TriBool _fnResult;
		try {
			_fnResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException("Error: Couldn't read the result of the function! Aborting!");
		}

		m_funcWriter.cleanUpAfterFunctionEvaluated(_wmc.m_address);
		
		log("going to write back the comparison results...");
		updateScoreWME(_comaRsnrFn.m_add_info_ptr.m_address,_fnResult);
		log(" successfully exiting processFunctionEvaluated()...");
		return;
	}


	@Override
	protected void dispatchComparison(String _comparisonID) {
		// TODO Auto-generated method stub

	}

	@Override
	protected TriBool executeComparison() {
		// TODO Auto-generated method stub
		return null;
	}


}
