package coma.aux;

import java.io.StringWriter;

import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.ReasonerRelation;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.abstr.WorkingMemoryReaderWriterProcess;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

import ComaData.ComaConcept;
import ComaData.ComaInstance;
import ComaData.ComaReasonerFunction;
import ComaData.ComaReasonerFunctionType;
import ComaData.ComaRelation;
import ComaData.StringWrapper;
import ComaData.TriBoolResult;

public class ComaFunctionWriter {

	private String m_subarchitectureID;
	private PrivilegedManagedProcess m_parentcomponent;
	
	public static enum ConceptQueryRestrictor {
		ALL,
		DIRECT,
		MOSTSPECIFIC,
		BASICLEVEL
	}
	
	public ComaFunctionWriter(PrivilegedManagedProcess _parentComponent) {
		this.m_parentcomponent = _parentComponent;
		this.m_subarchitectureID = m_parentcomponent.getSubarchitectureID();
	}
	
	/**
	 * Use this method when including this helper class from outside coma.sa!
	 * Per default, it is assumed that the parent component's SA is coma.sa.
	 * This method overrides the default assumption!
	 * 
	 * @param _comaSAID
	 */
	public void setComaSubarchitectureID(String _comaSAID) {
		this.m_subarchitectureID = _comaSAID;
	}
	
	/**
	 * This method tests whether a given concept already exists on WM.
	 * If so, it returns the corresponding WME.
	 * Otherwise, it returns null.
	 *
	 * @param _con
	 * @return
	 */
	private WorkingMemoryPointer getConceptWMP(ReasonerConcept _con) {
		try {
//			m_parentcomponent.log(m_parentcomponent);
//			m_parentcomponent.log(m_subarchitectureID);
//			m_parentcomponent.log(m_parentcomponent.getWorkingMemoryEntries(m_subarchitectureID, ComaConcept.class).length);
			for (CASTData<?> _currDataCon : m_parentcomponent.getWorkingMemoryEntries(m_subarchitectureID, ComaConcept.class)) {
				if (((((ComaConcept) _currDataCon.getData()).m_namespace).equals(_con.getNamespace())) 
						&& ((((ComaConcept) _currDataCon.getData()).m_sep).equals(_con.getSep()))
						&& ((((ComaConcept) _currDataCon.getData()).m_name).equals(_con.getName()))) {
					return new WorkingMemoryPointer(_currDataCon.getType(),
							new WorkingMemoryAddress(_currDataCon.getID(), m_subarchitectureID));
				}
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * This method tests whether a given instance already exists on WM.
	 * If so, it returns the corresponding WME.
	 * Otherwise, it returns null.
	 *
	 * @param _ins
	 * @return
	 */
	private WorkingMemoryPointer getInstanceWMP(ReasonerInstance _ins) {
		try {
//			m_parentcomponent.log("Trying to get Instance WMP for " + _ins + "from WM.");
//			m_parentcomponent.log("Loading all Instance WMEs.");
			for (CASTData<?> _currDataIns : m_parentcomponent.getWorkingMemoryEntries(m_subarchitectureID, ComaInstance.class)) {
//				m_parentcomponent.log("current Instance WME = " + ((ComaInstance) _currDataIns.getData()).m_name);
				
//				m_parentcomponent.log("Comparing: current **VS** required:");
//				m_parentcomponent.log("Namespace: " + ((ComaInstance) _currDataIns.getData()).m_namespace + " **VS ** "+ _ins.getNamespace());
//				m_parentcomponent.log("Separator: " + ((ComaInstance) _currDataIns.getData()).m_sep + " **VS ** "+ _ins.getSep());
//				m_parentcomponent.log("     Name: " + ((ComaInstance) _currDataIns.getData()).m_name + " **VS ** "+ _ins.getName());
				
				if (((((ComaInstance) _currDataIns.getData()).m_namespace).equals(_ins.getNamespace())) 
					&& ((((ComaInstance) _currDataIns.getData()).m_sep).equals(_ins.getSep()))
					&& ((((ComaInstance) _currDataIns.getData()).m_name).equals(_ins.getName()))) {
//					m_parentcomponent.log("FOUND A MATCH!");
					return new WorkingMemoryPointer(_currDataIns.getType(),
							new WorkingMemoryAddress(_currDataIns.getID(), m_subarchitectureID));
				}
//				m_parentcomponent.log("Not matching...");
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
	
	
	/**
	 * This method will query the ComaReasoner for all instances
	 * that are related to a given instance by a given relation.
	 * 
	 * It first gets the WME for the instance in question. If the coma WM
	 * does not contain such an instance, an exception is thrown.
	 * If, however, a suitable instance is found, this method puts a new
	 * ComaReasonerFunction of type GetRelatedInstances onto the coma WM,
	 * and registers an appropriate filter for the result callback.
	 * The relation is stored as a torso/dummy ComaRelation in the addInfoPtr
	 * field and is deleted afterwards in the callback method.
	 *
	 * @param _ins
	 * @throws InstanceUnknownException
	 */
	public String getRelatedInstances(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _ins, ReasonerRelation _rel)  throws InstanceUnknownException{
		
		try {
			WorkingMemoryPointer _insptr = getInstanceWMP(_ins);
			if (_insptr==null) {
				throw new InstanceUnknownException(_ins.getFullName());
			}
			else {
				String _relDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_relDataID, 
						new ComaRelation(_rel.getRelationNamespace(), _rel.getRelationSep(), _rel.getRelationName(), 
								new WorkingMemoryPointer("",new WorkingMemoryAddress("","")), 
								new WorkingMemoryPointer("",new WorkingMemoryAddress("",""))) ,
								OperationMode.BLOCKING);
				
				WorkingMemoryPointer _relptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaRelation.class), 
						new WorkingMemoryAddress(_relDataID, m_subarchitectureID));
	
				ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.GetRelatedInstances,
						_insptr,
						new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
						new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
						_relptr);
	
				String _reqID = m_parentcomponent.newDataID();
				if (_receiver!=null) m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID,
						m_subarchitectureID, WorkingMemoryOperation.OVERWRITE),
						_receiver);
				m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
				return _reqID;
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * This method will query the ComaReasoner for all related instances
	 * of a given instance.
	 * 
	 * It first gets the WME for the instance in question. If the coma WM
	 * does not contain such an instance, an exception is thrown.
	 * If, however, a suitable instance is found, this method puts a new
	 * ComaReasonerFunction of type GetRelatedInstances onto the coma WM,
	 * and registers an appropriate filter for the result callback.
	 *
	 * @param _ins
	 * @throws InstanceUnknownException
	 */
	public String getRelatedInstances(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _ins)  throws InstanceUnknownException{
		
		try {
			WorkingMemoryPointer _insptr = getInstanceWMP(_ins);
			if (_insptr==null) {
				throw new InstanceUnknownException(_ins.getFullName());
			}
			else {
				ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.GetRelatedInstances,
						_insptr,
						new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
						new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
						new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
				
				String _reqID = m_parentcomponent.newDataID();
				if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(
						_reqID, m_subarchitectureID, WorkingMemoryOperation.OVERWRITE),
						_receiver						);
				m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
				return _reqID;
			}
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * TODO adapt text!
	 * This method will query the ComaReasoner for all concepts of a given instance.
	 * 
	 * It first gets the WME for the instance in question. If the coma WM
	 * does not contain such an instance, an exception is thrown.
	 * If, however, a suitable instance is found, this method puts a new
	 * ComaReasonerFunction of type GetAllConcepts onto the coma WM,
	 * and registers an appropriate filter for the result callback.
	 *
	 * @param _ins
	 * @param _quantification
	 */
	public String getAllInstances(WorkingMemoryChangeReceiver _receiver, ReasonerConcept _con)  {  //, ConceptQueryRestrictor _quantification) throws InstanceUnknownException{
		
		try {
			WorkingMemoryPointer _conptr = getConceptWMP(_con);
			if (_conptr==null) {
				String _conDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_conDataID, new ComaConcept(_con.getNamespace(), _con.getSep(), _con.getName()), OperationMode.BLOCKING);
				_conptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_conDataID, m_subarchitectureID));
			}
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.GetAllInstances,
					_conptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(
					_reqID, m_subarchitectureID, WorkingMemoryOperation.OVERWRITE),
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * This method will query the ComaReasoner for all concepts of a given instance.
	 * 
	 * It first gets the WME for the instance in question. If the coma WM
	 * does not contain such an instance, an exception is thrown.
	 * If, however, a suitable instance is found, this method puts a new
	 * ComaReasonerFunction of type GetAllConcepts onto the coma WM,
	 * and registers an appropriate filter for the result callback.
	 *
	 * @param _ins
	 * @param _quantification
	 * @throws InstanceUnknownException
	 */
	public String getConcepts(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _ins, ConceptQueryRestrictor _quantification) throws InstanceUnknownException{
		
		try {
			WorkingMemoryPointer _insptr = getInstanceWMP(_ins);
			if (_insptr==null) {
				throw new InstanceUnknownException(_ins.getFullName());
			}
			ComaReasonerFunction _req = new ComaReasonerFunction(
					(_quantification.equals(ConceptQueryRestrictor.ALL) ? ComaReasonerFunctionType.GetAllConcepts
							: (_quantification.equals(ConceptQueryRestrictor.DIRECT) ? ComaReasonerFunctionType.GetAllDirectConcepts
									: (_quantification.equals(ConceptQueryRestrictor.MOSTSPECIFIC) ? ComaReasonerFunctionType.GetMostSpecificConcepts
											: (_quantification.equals(ConceptQueryRestrictor.BASICLEVEL) ? ComaReasonerFunctionType.GetBasicLevelConcepts
													: ComaReasonerFunctionType.GetAllConcepts)))), // default fall-back...
													_insptr,
													new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
													new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
													new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * This method adds a ComaRelation WME and two ComaInstance WMEs to Coma WM.
	 * The relation contains the pointers to the instance WMEs.
	 * It also writes a ComaReasonerFunction of type AddRelation that points
	 * to the relation WME.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and result WMEs.
	 * 
	 * @param _ins1
	 * @param _ins2
	 * @param _rel
	 */
	public String addInstanceInstanceRelation(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _ins1, ReasonerInstance _ins2, ReasonerRelation _rel) {

		try {
			WorkingMemoryPointer _ins1ptr = getInstanceWMP(_ins1);
			if (_ins1ptr==null) {
				String _ins1DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_ins1DataID, new ComaInstance(_ins1.getNamespace(),_ins1.getSep(),_ins1.getName(), new String[0], new String[0]), OperationMode.BLOCKING);
				_ins1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class), 
						new WorkingMemoryAddress(_ins1DataID, m_subarchitectureID));
			}

			WorkingMemoryPointer _ins2ptr = getInstanceWMP(_ins2);
			if (_ins2ptr==null) {
				String _ins2DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_ins2DataID, new ComaInstance(_ins2.getNamespace(), _ins2.getSep(), _ins2.getName(), new String[0], new String[0]), OperationMode.BLOCKING);
				_ins2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class), 
						new WorkingMemoryAddress(_ins2DataID, m_subarchitectureID)); 
			}

			String _relDataID = m_parentcomponent.newDataID();
			m_parentcomponent.addToWorkingMemory(_relDataID, new ComaRelation(_rel.getRelationNamespace(), _rel.getRelationSep(), _rel.getRelationName(),
					_ins1ptr,
					_ins2ptr), OperationMode.BLOCKING);

			WorkingMemoryPointer _arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaRelation.class), 
					new WorkingMemoryAddress(_relDataID, m_subarchitectureID));

			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.AddRelation,_arg1ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));

			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * This method adds a ComaInstance WME and a ComaConcept WME to Coma WM.
	 * It also writes a ComaReasonerFunction of type AddInstance that points
	 * to the instance and concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _instance
	 * @param _concept
	 */
	public String addInstanceConceptAssertion(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _instance, ReasonerConcept _concept) {
		
		try {
			WorkingMemoryPointer _arg1ptr = getInstanceWMP(_instance);
			if (_arg1ptr==null) {
				String _insDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_insDataID, new ComaInstance(_instance.getNamespace(), 
						_instance.getSep(), _instance.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}
	
			WorkingMemoryPointer _arg2ptr = getConceptWMP(_concept);
			if (_arg2ptr==null) {
				String _conDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_conDataID, new ComaConcept(_concept.getNamespace(),_concept.getSep(),_concept.getName()), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_conDataID, m_subarchitectureID));
			}
			
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.AddInstance,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	/**
	 * This method adds a ComaInstance WME and a String WME to Coma WM.
	 * It also writes a ComaReasonerFunction of type AddInstanceName that points
	 * to the instance and given name WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _instance
	 * @param _name
	 */
	public String addInstanceName(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _instance, String _name) {
		
		try {
			WorkingMemoryPointer _arg1ptr = getInstanceWMP(_instance);
			if (_arg1ptr==null) {
				String _insDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_insDataID, new ComaInstance(_instance.getNamespace(), 
						_instance.getSep(), _instance.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}
	
			String _nameDataID = m_parentcomponent.newDataID();
			m_parentcomponent.addToWorkingMemory(_nameDataID, new StringWrapper(_name), OperationMode.BLOCKING);
			WorkingMemoryPointer _arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(StringWrapper.class),
					new WorkingMemoryAddress(_nameDataID, m_subarchitectureID));
			
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.AddInstanceName,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	/**
	 * This method adds a ComaInstance WME and a String WME to Coma WM.
	 * It also writes a ComaReasonerFunction of type AddInstanceName that points
	 * to the instance and number tag WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * NB for WM interaction, the number tag is treated as a string.
	 * Internally, the reasoner treats it as an integer datatype though!
	 * 
	 * @param _instance
	 * @param _number
	 */
	public String addInstanceNumberTag(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _instance, Integer _number) {
		
		try {
			WorkingMemoryPointer _arg1ptr = getInstanceWMP(_instance);
			if (_arg1ptr==null) {
				String _insDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_insDataID, new ComaInstance(_instance.getNamespace(), 
						_instance.getSep(), _instance.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}
	
			String _nameDataID = m_parentcomponent.newDataID();
			m_parentcomponent.addToWorkingMemory(_nameDataID, new StringWrapper(_number.toString()), OperationMode.BLOCKING);
			WorkingMemoryPointer _arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(StringWrapper.class),
					new WorkingMemoryAddress(_nameDataID, m_subarchitectureID));
			
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.AddInstanceNumberTag,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	/**
	 * This method adds a ComaInstance WME to Coma WM.
	 * It also writes a ComaReasonerFunction of type DeleteInstance that points
	 * to the instance WME.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _instance
	 */
	public String deleteInstance(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _instance) {
		
		try {
			WorkingMemoryPointer _arg1ptr = getInstanceWMP(_instance);
			if (_arg1ptr==null) {
				String _insDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_insDataID, new ComaInstance(_instance.getNamespace(), 
						_instance.getSep(), _instance.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}
	
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.DeleteInstance,
					_arg1ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}


	
	/** 
	 * This method adds a ComaInstance WME and a ComaConcept WME to Coma WM.
	 * It also writes a ComaReasonerFunction of type IsInstanceOf that points
	 * to the instance and concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _instance
	 * @param _concept
	 */
	public String askInstanceOf(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _instance, ReasonerConcept _concept) {
		
		try {
			WorkingMemoryPointer _arg1ptr = getInstanceWMP(_instance);
			if (_arg1ptr==null) {
				String _insDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_insDataID, new ComaInstance(_instance.getNamespace(), 
						_instance.getSep(), _instance.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}
	
			WorkingMemoryPointer _arg2ptr = getConceptWMP(_concept);
			if (_arg2ptr==null) {
				String _conDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_conDataID, new ComaConcept(_concept.getNamespace(),_concept.getSep(),_concept.getName()), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_conDataID, m_subarchitectureID));
			}
			
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.IsInstanceOf,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	/**
	 * This method adds two ComaConcept WMEs to Coma WM.
	 * It also writes a ComaReasonerFunction of type AreConsEquivalent that points
	 * to the concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _con1
	 * @param _con2
	 */
	public String askConsEqui(WorkingMemoryChangeReceiver _receiver, ReasonerConcept _con1, ReasonerConcept _con2) {

		try {
			WorkingMemoryPointer _arg1ptr = getConceptWMP(_con1);
			if (_arg1ptr==null) {
				String _con1DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con1DataID, new ComaConcept(_con1.getNamespace(), 
						_con1.getSep(), _con1.getName()), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con1DataID, m_subarchitectureID));
			}

			WorkingMemoryPointer _arg2ptr = getConceptWMP(_con2);
			if (_arg2ptr==null) {
				String _con2DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con2DataID, new ComaConcept(_con2.getNamespace(),_con2.getSep(),_con2.getName()), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con2DataID, m_subarchitectureID));
			}

			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.AreConsEquivalent,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));

			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * This method adds two ComaConcept WMEs to Coma WM.
	 * It also writes a ComaReasonerFunction of type IsConSubcon that points
	 * to the concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _con1
	 * @param _con2
	 */
	public String askSubCon(WorkingMemoryChangeReceiver _receiver, ReasonerConcept _con1, ReasonerConcept _con2) {

		try {
			WorkingMemoryPointer _arg1ptr = getConceptWMP(_con1);
			if (_arg1ptr==null) {
				String _con1DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con1DataID, new ComaConcept(_con1.getNamespace(), 
						_con1.getSep(), _con1.getName()), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con1DataID, m_subarchitectureID));
			}

			WorkingMemoryPointer _arg2ptr = getConceptWMP(_con2);
			if (_arg2ptr==null) {
				String _con2DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con2DataID, new ComaConcept(_con2.getNamespace(),_con2.getSep(),_con2.getName()), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con2DataID, m_subarchitectureID));
			}

			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.IsConSubcon,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));

			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	/**
	 * This method adds two ComaConcept WMEs to Coma WM.
	 * It also writes a ComaReasonerFunction of type IsConSupercon that points
	 * to the concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _con1
	 * @param _con2
	 */
	public String askSuperCon(WorkingMemoryChangeReceiver _receiver, ReasonerConcept _con1, ReasonerConcept _con2) {

		try {
			WorkingMemoryPointer _arg1ptr = getConceptWMP(_con1);
			if (_arg1ptr==null) {
				String _con1DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con1DataID, new ComaConcept(_con1.getNamespace(), 
						_con1.getSep(), _con1.getName()), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con1DataID, m_subarchitectureID));
			}

			WorkingMemoryPointer _arg2ptr = getConceptWMP(_con2);
			if (_arg2ptr==null) {
				String _con2DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con2DataID, new ComaConcept(_con2.getNamespace(),_con2.getSep(),_con2.getName()), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con2DataID, m_subarchitectureID));
			}

			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.IsConSupercon,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));

			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * This method adds two ComaConcept WMEs to Coma WM.
	 * It also writes a ComaReasonerFunction of type CompareCons that points
	 * to the concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _con1
	 * @param _con2
	 */
	public String compareCons(WorkingMemoryChangeReceiver _receiver, ReasonerConcept _con1, ReasonerConcept _con2, WorkingMemoryPointer _addInfoWMP) {

		try {
			WorkingMemoryPointer _arg1ptr = getConceptWMP(_con1);
			if (_arg1ptr==null) {
				String _con1DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con1DataID, new ComaConcept(_con1.getNamespace(), 
						_con1.getSep(), _con1.getName()), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con1DataID, m_subarchitectureID));
			}

			WorkingMemoryPointer _arg2ptr = getConceptWMP(_con2);
			if (_arg2ptr==null) {
				String _con2DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con2DataID, new ComaConcept(_con2.getNamespace(),_con2.getSep(),_con2.getName()), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con2DataID, m_subarchitectureID));
			}

			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.CompareCons,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					(_addInfoWMP!=null ? _addInfoWMP : new WorkingMemoryPointer("",new WorkingMemoryAddress("",""))));

			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	/**
	 * This method adds a ComaInstance WME and a ComaConcept WME to Coma WM.
	 * It also writes a ComaReasonerFunction of type IsInstanceOf that points
	 * to the instance and concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _instance
	 * @param _concept
	 */
	public String compareInstanceConcept(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _instance, ReasonerConcept _concept, WorkingMemoryPointer _addInfoWMP) {
		
		try {
			WorkingMemoryPointer _arg1ptr = getInstanceWMP(_instance);
			if (_arg1ptr==null) {
				String _insDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_insDataID, new ComaInstance(_instance.getNamespace(), 
						_instance.getSep(), _instance.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}
	
			WorkingMemoryPointer _arg2ptr = getConceptWMP(_concept);
			if (_arg2ptr==null) {
				String _conDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_conDataID, new ComaConcept(_concept.getNamespace(),_concept.getSep(),_concept.getName()), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_conDataID, m_subarchitectureID));
			}
			
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.IsInstanceOf,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					(_addInfoWMP!=null ? _addInfoWMP : new WorkingMemoryPointer("",new WorkingMemoryAddress("",""))));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	/**
	 * This method adds one ComaConcept WME to Coma WM.
	 * It also writes a ComaReasonerFunction of type GetObjectMobility that points
	 * to the concept WMEs.
	 * It waits for the ComaReasoner to process the function and then
	 * deletes the function and the result TriBool WME!
	 * 
	 * @param _con
	 */
	public String getMobility(WorkingMemoryChangeReceiver _receiver, ReasonerConcept _con) {

		try {
			WorkingMemoryPointer _arg1ptr = getConceptWMP(_con);
			if (_arg1ptr==null) {
				String _con1DataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_con1DataID, new ComaConcept(_con.getNamespace(), 
						_con.getSep(), _con.getName()), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaConcept.class),
						new WorkingMemoryAddress(_con1DataID, m_subarchitectureID));
			}

			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.GetObjectMobility,
					_arg1ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));

			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	/**
	 * This method will query the ComaReasoner for all typical objects 
	 * for a given concept.
	 *
	 * @param _con
	 */
	public String getTypicalObjs(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _ins) {
		try {
			WorkingMemoryPointer _insptr = getInstanceWMP(_ins);
			if (_insptr==null) {
				String _insDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_insDataID, new ComaInstance(_ins.getNamespace(), _ins.getSep(), _ins.getName(),new String[0],new String[0]), OperationMode.BLOCKING);
				_insptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.GetTypicalObjects,
					_insptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));

			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(
					_reqID, m_subarchitectureID, WorkingMemoryOperation.OVERWRITE),
					_receiver);
			m_parentcomponent.addToWorkingMemory(_reqID, _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}
	
	/** 
	 * TODO
	 * 
	 * @param _instance
	 */
	public String generateRefEx(WorkingMemoryChangeReceiver _receiver, ReasonerInstance _intRef, ReasonerInstance _origin, String _saID) {
		log("generateRefEx called");
		try {
			WorkingMemoryPointer _arg1ptr = getInstanceWMP(_intRef);
			if (_arg1ptr==null) {
				String _insDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_insDataID, new ComaInstance(_intRef.getNamespace(), 
						_intRef.getSep(), _intRef.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg1ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_insDataID, m_subarchitectureID));
			}
	
			WorkingMemoryPointer _arg2ptr = getInstanceWMP(_origin);
			if (_arg2ptr==null) {
				String _oriDataID = m_parentcomponent.newDataID();
				m_parentcomponent.addToWorkingMemory(_oriDataID, new ComaInstance(_origin.getNamespace(),
						_origin.getSep(),_origin.getName(),new String[0], new String[0]), OperationMode.BLOCKING);
				_arg2ptr = new WorkingMemoryPointer(CASTUtils.typeName(ComaInstance.class),
						new WorkingMemoryAddress(_oriDataID, m_subarchitectureID));
			}
			
			ComaReasonerFunction _req = new ComaReasonerFunction(ComaReasonerFunctionType.GenerateRefEx,
					_arg1ptr,
					_arg2ptr,
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")),
					new WorkingMemoryPointer("",new WorkingMemoryAddress("","")));
	
			String _reqID = m_parentcomponent.newDataID();
			if (_receiver!=null)  m_parentcomponent.addChangeFilter(ChangeFilterFactory.createAddressFilter(_reqID, 
					m_subarchitectureID, 
					WorkingMemoryOperation.OVERWRITE), 
					_receiver);
			m_parentcomponent.addToWorkingMemory(new WorkingMemoryAddress(_reqID, _saID), _req, OperationMode.BLOCKING);
			return _reqID;
		} catch (SubarchitectureProcessException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return null;
	}

	
	
	/**
	 * This method will clean up the coma WM after a ComaReasonerFunction has been evaluated.
	 * It will delete: 
	 * <li> the original function struct
	 * <li> the entries pointed to by arg1ptr and arg2ptr iff they are neither of type 
	 *  	ComaConcept or ComaInstance, nor empty (obviously)
	 * <li> the entry pointed to by add_info_ptr iff it is inside coma WM
	 * <li> the entry pointed to by resultprt
	 * 
	 * @param _functionWMP
	 */
	public void cleanUpAfterFunctionEvaluated(WorkingMemoryAddress _functionWMA) {
		try {
			ComaReasonerFunction _function = (ComaReasonerFunction) m_parentcomponent.getWorkingMemoryEntry(_functionWMA).getData();
			if (_function.m_functiontype.value()==ComaReasonerFunctionType._DeleteInstance &&
					!_function.m_arg1ptr.m_address.m_id.equals("") &&
					_function.m_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) {
				
				TriBool _fnResult;
				try {
					_fnResult = ((TriBoolResult) m_parentcomponent.getWorkingMemoryEntry(
							_function.m_resultptr.m_address).getData()).m_tribool;
				} catch (SubarchitectureProcessException e) {
					e.printStackTrace();
					throw new RuntimeException("Error: Couldn't read the result of the function! Aborting!");
				}

				if (_fnResult.equals(TriBool.triTrue)) {
					m_parentcomponent.log("deleting arg1 at address "+_function.m_arg1ptr.m_address.m_id);
					m_parentcomponent.deleteFromWorkingMemory(_function.m_arg1ptr.m_address.m_id,OperationMode.BLOCKING);
				} else {
					m_parentcomponent.log("got an unsuccessful delete instance function; so I am not cleaning up the instance WME!");
				}				
			}
			
    		m_parentcomponent.log("deleting original COMA REASONER FUNCTION from WM.");
    		m_parentcomponent.deleteFromWorkingMemory(_functionWMA.m_id, OperationMode.BLOCKING);

        	// now that we're here we can safely delete the concept WMEs
        	m_parentcomponent.log("deleting obsolete working memory entries from WM!");

        	if (!_function.m_arg1ptr.m_address.m_id.equals("") &&
        			(!_function.m_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) &&
        			(!_function.m_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class)))) {
        		m_parentcomponent.log("deleting arg1 of type " + _function.m_arg1ptr.m_type + " at address "+_function.m_arg1ptr.m_address.m_id);
        		m_parentcomponent.deleteFromWorkingMemory(_function.m_arg1ptr.m_address.m_id);
        	}
        	if (!_function.m_arg2ptr.m_address.m_id.equals("") &&
        			(!_function.m_arg1ptr.m_type.equals(CASTUtils.typeName(ComaInstance.class))) &&
        			(!_function.m_arg1ptr.m_type.equals(CASTUtils.typeName(ComaConcept.class)))) {
        		m_parentcomponent.log("deleting arg2 of type " + _function.m_arg2ptr.m_type + " at address "+_function.m_arg2ptr.m_address.m_id);
        		m_parentcomponent.deleteFromWorkingMemory(_function.m_arg2ptr.m_address.m_id);
        	}

        	// add_info_ptr is used for relations and feature comparisons
        	// it is okay to delete relations
        	// normally, feature comparison tasks are on binding WM -- so not on "my own" WM;
        	// but in the coma tests, feature comparison tasks are written to coma WM for testing
        	// purposes... so I need to explicitly check for the type 
        	if (!_function.m_add_info_ptr.m_address.m_id.equals("")) {
    			if (_function.m_add_info_ptr.m_address.m_subarchitecture.
    					equals(m_parentcomponent.getSubarchitectureID()) &&
    					_function.m_add_info_ptr.m_type.equals(CASTUtils.typeName(ComaRelation.class))) {
    				m_parentcomponent.log("deleting add_info of type " + _function.m_add_info_ptr.m_type + " at address "+_function.m_add_info_ptr.m_address.m_id);
    				m_parentcomponent.deleteFromWorkingMemory(_function.m_add_info_ptr.m_address.m_id);
    			}
    		}

        	// I assume the following structs can be pointed to:
        	// TriBoolResult -- okay to delete
        	// StringWrapper -- okay to delete
        	// ComaConcept[] -- will also delete contained ComaConcepts -- okay
        	// WorkingMemoryPointer[] -- pointing to ComaInstances, which will remain untouched -- okay
        	if (!_function.m_resultptr.m_address.m_id.equals("")) {
    			m_parentcomponent.log("deleting result of type " + _function.m_resultptr.m_type + " at address "+_function.m_resultptr.m_address.m_id);
    			m_parentcomponent.deleteFromWorkingMemory(_function.m_resultptr.m_address.m_id); 
    		}
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			m_parentcomponent.log("Error cleaning up coma reasoner function! Exiting!");
			throw new RuntimeException(e);
		}
	}
	
	private void log(String _logMsg) {
		m_parentcomponent.log("[ComaFunctionWriter] " + _logMsg);
	}
}