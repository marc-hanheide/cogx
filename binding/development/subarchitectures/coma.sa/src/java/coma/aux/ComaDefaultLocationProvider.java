package coma.aux;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInterface;
import org.cognitivesystems.reasoner.crowl.CrowlWrapper;

import planning.autogen.Action;
import planning.autogen.PlanningStatus;

import ComaData.ComaConcept;
import ComaData.ComaRelation;
import ComaData.GenerateComaProxies;

import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.rdf.model.Resource;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;

import coma.components.ComaReasoner;

public class ComaDefaultLocationProvider {

	private ComaReasoner m_parentComponent;
	private WorkingMemoryChange m_actionWMC;

	public ComaDefaultLocationProvider(ComaReasoner _parentReasoner, WorkingMemoryChange _actionWMC) {
		m_parentComponent = _parentReasoner;
		m_actionWMC = _actionWMC;
		log("new DefaultLocProvider initialized for action " + _actionWMC.m_address.m_id);
	}
	
	private ReasonerInterface getReasoner() {
		return m_parentComponent.getReasoner();
	}

	private void log(String _logMsg) {
		m_parentComponent.log("ReferenceResolver: " + _logMsg);
	}

	
	public void provideDefaultLocForProxy(WorkingMemoryAddress _proxyAddress, Set<ReasonerConcept> _allRCons) {
		// 	determine default location:
//		SELECT DISTINCT ?defaultLoc WHERE {
//			?defaultLoc rdfs:subClassOf ?blankNode.
//			?defaultLoc rdfs:subClassOf oe:Area.
//			FILTER (!isBlank(?defaultLoc)).
//			FILTER (?defaultLoc != owl:Nothing).
//			FILTER (isBlank(?blankNode)).
//			?blankNode owl:someValuesFrom ?defaultObjClass.
//			{ 
//			{ *CONi* rdfs:subClassOf ?defaultObjClass. }
//			UNION { *CONi+1* rdfs:subClassOf ?defaultObjClass. }
//			...							
//			UNION {*CONn* rdfs:subClassOf ?defaultObjClass. }
//			} 
//			}
		
		String _query = "SELECT DISTINCT ?defaultLoc WHERE { " +
			" ?defaultLoc rdfs:subClassOf ?blankNode. " +
			" ?defaultLoc rdfs:subClassOf oe:Area. " +
			" FILTER (!isBlank(?defaultLoc)). " +
			" FILTER (?defaultLoc != owl:Nothing). " +
			" FILTER (isBlank(?blankNode)). " +
			" ?blankNode owl:someValuesFrom ?defaultObjClass. " +
			" { "; 
		int coni = 0;
		for (ReasonerConcept _rCon : _allRCons) {
			if (coni>0) _query += " UNION ";
			_query += "{ " + _rCon.getFullName() + " rdfs:subClassOf ?defaultObjClass. } ";
			coni++;
		}
		_query += " } } ";
		log("going to execute query: " + _query);
		ResultSet _results = ((CrowlWrapper) getReasoner()).executeSPARQLQuery(_query);
		
		
		boolean _success = false;
		ArrayList<WorkingMemoryPointer> _conWMPS = new ArrayList<WorkingMemoryPointer>();
		
		while (_results.hasNext()) {
			QuerySolution _qs = (QuerySolution) _results.next();
			log("current binding:");
			Resource _currAnswerResource = _qs.getResource("?defaultLoc");
			log("?defaultLoc is bound to " + _currAnswerResource.toString());
			
			WorkingMemoryPointer _currConWMP = null;
			try {
				_currConWMP = m_parentComponent.getConceptWMP(
						OntologyMemberFactory.createConcept(
								"oe", ":", 
								_currAnswerResource.getLocalName()));
				if (_currConWMP==null) {
					// concept not yet existant on coma WM
					ComaConcept _newCon = new ComaConcept("oe", ":", _currAnswerResource.getLocalName());
					String _newConID = m_parentComponent.newDataID();
					m_parentComponent.addToWorkingMemory(_newConID, _newCon);
					_currConWMP = new WorkingMemoryPointer(CASTUtils.typeName(_newCon), 
							new WorkingMemoryAddress(_newConID, m_parentComponent.getSubarchitectureID()));
				}
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			_conWMPS.add(_currConWMP);
		}
		if (_conWMPS.isEmpty()) _success=false;
		else _success=true;

		if (_success) {
			// generate the ID of the GenerateComaProxies struct:
			String _generationTaskID = m_parentComponent.newDataID();
			
			// generate the list of concepts for the proxy in question:
			ArrayList<String> _defCons = new ArrayList<String>();
			for (ReasonerConcept _rCon : _allRCons) {
				_defCons.add(_rCon.getName());
			}

			ArrayList<WorkingMemoryPointer> _relWMPs = new ArrayList<WorkingMemoryPointer>();
			// generate the list of relations to be created
			for (WorkingMemoryPointer _conWMP : _conWMPS) {
				ComaRelation _currRel = new ComaRelation("","","position", 
						new WorkingMemoryPointer(CASTUtils.typeName(GenerateComaProxies.class),
								new WorkingMemoryAddress(_generationTaskID, 
										m_parentComponent.getSubarchitectureID())),
								_conWMP);
				String _currRelID = m_parentComponent.newDataID();
				try {
					m_parentComponent.addToWorkingMemory(_currRelID, _currRel);
				} catch (AlreadyExistsOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (SubarchitectureProcessException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				_relWMPs.add(new WorkingMemoryPointer(CASTUtils.typeName(_currRel),
						new WorkingMemoryAddress(_currRelID,m_parentComponent.getSubarchitectureID())));
			}
			
			// OK, now I should generate them proxies
			// TODO represent all involved entities and their relations				
			WorkingMemoryPointer[] _instanceWMPArray = new WorkingMemoryPointer[0];
			WorkingMemoryPointer[] _conceptWMPArray  = new WorkingMemoryPointer[_conWMPS.size()]; 
			int i=0;
			for (WorkingMemoryPointer pointer : _conWMPS) {
				_conceptWMPArray[i] = pointer;
				i++;
			}

			WorkingMemoryPointer[] _relationWMPArray = new WorkingMemoryPointer[_relWMPs.size()];
			int j=0;
			for (WorkingMemoryPointer pointer : _relWMPs) {
				_relationWMPArray[j] = pointer;
				j++;
			}
			
			String[] _defConStringArray = new String[_defCons.size()];
			int k=0;
			for (String defCon : _defCons) {
				_defConStringArray[k] = defCon;
				k++;
			}
			
			
			
			try {
				GenerateComaProxies _newProxyGenTask = new GenerateComaProxies(
						_instanceWMPArray, _conceptWMPArray, _relationWMPArray, _defConStringArray);

				m_parentComponent.addToWorkingMemory(_generationTaskID, _newProxyGenTask);
				m_parentComponent.addChangeFilter(
						ChangeFilterFactory.createAddressFilter(_generationTaskID, 
								m_parentComponent.getSubarchitectureID(),
								WorkingMemoryOperation.DELETE),
								new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
									WorkingMemoryChange _wmc) {
								proxiesAdded(_wmc);
							}});
				_success = true;
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				_success=false;
			}
		}
		
		// 
		if (!_success) {
			log("overwriting action cos it was unsuccessful!");
			Action _myAction;
			try {
				_myAction = (Action) m_parentComponent.getWorkingMemoryEntry(m_actionWMC.m_address).getData();
				_myAction.m_succeeded=TriBool.triFalse;
				m_parentComponent.overwriteWorkingMemory(m_actionWMC.m_address, 
						_myAction, OperationMode.BLOCKING);
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	private void proxiesAdded(WorkingMemoryChange _wmc) {
		log("all proxies created -- reporting back");
		Action _myAction;
		try {
			_myAction = (Action) m_parentComponent.getWorkingMemoryEntry(m_actionWMC.m_address).getData();
			_myAction.m_succeeded=TriBool.triTrue;
			_myAction.m_status=PlanningStatus.COMPLETE;
			m_parentComponent.overwriteWorkingMemory(m_actionWMC.m_address, 
					_myAction, OperationMode.BLOCKING);
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}


}
