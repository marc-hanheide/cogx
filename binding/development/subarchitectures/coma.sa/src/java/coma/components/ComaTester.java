package coma.components;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.TreeMap;
import java.util.Properties;

import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.reasoner.base.OntologyMemberFactory;

import BindingData.FeatureComparison;
import BindingData.FeatureComparisonTask;
import BindingData.FeaturePointer;
import BindingFeatures.AreaID;
import BindingFeatures.Concept;
import BindingFeaturesCommon.ParentFeature;


import ComaData.ComaConcept;
import ComaData.ComaInstance;
import ComaData.ComaReasonerFunction;
import ComaData.ComaReasonerFunctionType;
import ComaData.ComaRelation;
import ComaData.TriBoolResult;
import coma.aux.ComaFunctionWriter;
import coma.aux.ComaHelper;
import coma.aux.InstanceUnknownException;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTException;
import cast.core.CASTUtils;
import cast.core.data.CASTData;
import cast.testing.AbstractTester;

/** 
 * 
 *  Use this code as argument to the WM interaction helper methods!
 *  
  new WorkingMemoryChangeReceiver() {
  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
  		// log(CASTUtils.toString(_wmc));
  		handleComaReasonerFunctionChanged(_wmc);
  	}
  }
 */

/**
 * @author zender
 *
 */
public class ComaTester extends AbstractTester {
	
	
	private String m_ontologyNamespace;
	private String m_ontologySep;

	
	public ComaTester(String _id) {
        super(_id);
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	}
	
	private String m_testfile;
	
	
    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        super.start();
        // own code goes in here....
        // nice piece of code to trace WM deletes, just uncomment to use:
//        try {
//        	addChangeFilter(WorkingMemoryOperation.DELETE, true, new WorkingMemoryChangeReceiver() {
//                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
//                    	log( _wmc.m_src + " deleted the WME at address " + _wmc.m_address.m_id);
//                    }
//                });
//        }
//        catch (SubarchitectureProcessException e) {
//            e.printStackTrace();
//        }
    }

    /*
    * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
    */
   @Override
   public void configure(Properties _config) {
	   super.configure(_config);
   	
       if (_config.containsKey("--onto_ns")) {
           this.m_ontologyNamespace = _config.getProperty("--onto_ns");
       }
       else {
//           this.m_ontologyNamespace = "http://www.dfki.de/cosy/officeenv.owl";
           this.m_ontologyNamespace = "oe";
       }
       if (_config.containsKey("--onto_sep")) {
           this.m_ontologySep= _config.getProperty("--onto_sep");
       }
       else {
//           this.m_ontologySep = "#";
           this.m_ontologySep = "oe";
       }
       log("Using namespace: " + m_ontologyNamespace);
       log("Using separator: " + m_ontologySep);

       if (_config.containsKey("--testfile")) {
    	   this.m_testfile= _config.getProperty("--testfile");
    	   log("Using testfile: " + m_testfile);
       } else {
    	   //TODO adapt!
    	   System.err.println("you need to specify a test file!");
    	   System.exit(1000);
       }
       FileInputStream fstream;
       try {
    	   fstream = new FileInputStream(m_testfile);
    	   // Get the object of DataInputStream
    	   DataInputStream in = new DataInputStream(fstream);
    	   BufferedReader br = new BufferedReader(new InputStreamReader(in));
    	   String strLine;
    	   int testI=0;
    	   //Read File Line By Line
    	   while ((strLine = br.readLine()) != null)   {
    		   try {
    			   ComaTest _currTest = new ComaTest(strLine, this);
    			   String _currTestID = "comaTest"+testI++; 
    			   super.registerTest(_currTestID, _currTest);
    			   super.queueTest(_currTestID);
    			   log("Test "+_currTestID +" = "+strLine);
    		   } catch (CASTException e) {
    			   e.printStackTrace();
    			   throw new RuntimeException("Error! Could not properly register and queue a test. Aborting!");
    		   }
    	   }
    	   //Close the input stream
    	   in.close();
       } catch (FileNotFoundException e) {
    	   e.printStackTrace();
    	   throw new RuntimeException("Error: File not found! Aborting!");
       }
       catch (IOException e) {
    	   e.printStackTrace();
    	   throw new RuntimeException("Error: I/O Exception. Aborting!");
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

	private class ComaTest extends AbstractTest {
		
		protected ComaTest(String _testline, ComaTester _parent) {
			super();
			this._strLine = _testline;
			this.m_reqToResultMap = new TreeMap<String, String>();
			this.m_oeMemberMaker = new OntologyMemberFactory(m_ontologyNamespace, m_ontologySep);
			this.m_funcWriter = new ComaFunctionWriter(_parent);
		}
		
		private String _strLine;
		private TreeMap<String,String> m_reqToResultMap;
		private OntologyMemberFactory m_oeMemberMaker;
		private ComaFunctionWriter m_funcWriter;
		
		@Override
		protected void startTest() {
			log("startTest()");
			if (_strLine.startsWith("instance")) {
				String[] _tokens = _strLine.split(",");
				ReasonerInstance _ins = m_oeMemberMaker.createInstance(_tokens[1]);
				ReasonerConcept _con = m_oeMemberMaker.createConcept(_tokens[2]);
				String _result = _tokens[3];
				m_reqToResultMap.put(m_funcWriter.addInstanceConceptAssertion(
						new WorkingMemoryChangeReceiver() {
				  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				  		handleComaReasonerFunctionChanged(_wmc);} },_ins, _con), _result);
			} else if (_strLine.startsWith("relation")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.addInstanceInstanceRelation(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createInstance(_tokens[1]), m_oeMemberMaker.createInstance(_tokens[2]), m_oeMemberMaker.createRelation(_tokens[3])),_tokens[4]);
			} else if (_strLine.startsWith("getcons")) {
				String[] _tokens = _strLine.split(",");
				ComaFunctionWriter.ConceptQueryRestrictor _res;
				if (_tokens[2].equals("ALL")) _res=ComaFunctionWriter.ConceptQueryRestrictor.ALL; 
				else if (_tokens[2].equals("MOSTSPECIFIC")) _res=ComaFunctionWriter.ConceptQueryRestrictor.MOSTSPECIFIC; 
				else if (_tokens[2].equals("DIRECT")) _res=ComaFunctionWriter.ConceptQueryRestrictor.DIRECT;
				else if (_tokens[2].equals("BASICLEVEL")) _res=ComaFunctionWriter.ConceptQueryRestrictor.BASICLEVEL;
				else _res=ComaFunctionWriter.ConceptQueryRestrictor.ALL;
				try {
					m_reqToResultMap.put(m_funcWriter.getConcepts(
							new WorkingMemoryChangeReceiver() {
					  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					  		handleComaReasonerFunctionChanged(_wmc);} }, 
					  		m_oeMemberMaker.createInstance(_tokens[1]), _res),_tokens[3]);
				} catch (InstanceUnknownException e) {
					e.printStackTrace();
					throw new RuntimeException("Error: instance unkown! Aborting!");
				}
			} else if (_strLine.startsWith("getallins")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.getAllInstances(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createConcept(_tokens[1])),_tokens[2]);
			} else if (_strLine.startsWith("getallrelins")) {
				String[] _tokens = _strLine.split(",");
				try {
					m_reqToResultMap.put(m_funcWriter.getRelatedInstances(
							new WorkingMemoryChangeReceiver() {
							  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
							  		handleComaReasonerFunctionChanged(_wmc);} },
							  		m_oeMemberMaker.createInstance(_tokens[1])),_tokens[2]);
				} catch (InstanceUnknownException e) {
					e.printStackTrace();
					throw new RuntimeException("Error: instance unkown! Aborting!");
				}
			} else if (_strLine.startsWith("getrelins")) {
				String[] _tokens = _strLine.split(",");
				try {
					m_reqToResultMap.put(m_funcWriter.getRelatedInstances(
							new WorkingMemoryChangeReceiver() {
							  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
							  		handleComaReasonerFunctionChanged(_wmc);} },
							  		m_oeMemberMaker.createInstance(_tokens[1]),m_oeMemberMaker.createRelation(_tokens[2])),_tokens[3]);
				} catch (InstanceUnknownException e) {
					e.printStackTrace();
					throw new RuntimeException("Error: instance unkown! Aborting!");
				}
			} else if (_strLine.startsWith("consequi")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.askConsEqui(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createConcept(_tokens[1]),m_oeMemberMaker.createConcept(_tokens[2])),_tokens[3]);
			} else if (_strLine.startsWith("issubcon")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.askSubCon(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createConcept(_tokens[1]),m_oeMemberMaker.createConcept(_tokens[2])),_tokens[3]);
			} else if (_strLine.startsWith("issupercon")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.askSuperCon(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createConcept(_tokens[1]),m_oeMemberMaker.createConcept(_tokens[2])),_tokens[3]);
			} else if (_strLine.startsWith("comparable")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.compareCons(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createConcept(_tokens[1]),m_oeMemberMaker.createConcept(_tokens[2]),null),_tokens[3]);
			} else if (_strLine.startsWith("mobility")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.getMobility(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createConcept(_tokens[1])),_tokens[2]);
			} else if (_strLine.startsWith("typicalobjs")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.getTypicalObjs(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createInstance(_tokens[1])),_tokens[2]);
			} else if (_strLine.startsWith("deleteins")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(m_funcWriter.deleteInstance(
						new WorkingMemoryChangeReceiver() {
						  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						  		handleComaReasonerFunctionChanged(_wmc);} },
						  		m_oeMemberMaker.createInstance(_tokens[1])),_tokens[2]);
			} else if (_strLine.startsWith("comparecons")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(poseComparisonTask(m_oeMemberMaker.createConcept(_tokens[1]),m_oeMemberMaker.createConcept(_tokens[2])),_tokens[3]);
			} else if (_strLine.startsWith("compareareacon")) {
				String[] _tokens = _strLine.split(",");
				m_reqToResultMap.put(poseComparisonTask(new AreaID(Integer.parseInt(_tokens[1]),
						new ParentFeature()),m_oeMemberMaker.createConcept(_tokens[2])),_tokens[3]);
			} else if (_strLine.startsWith("wait")){
				String[] _tokens = _strLine.split(",");
				log("Waiting for "+_tokens[1]+"seconds...");
				try {
					sleepProcess(Integer.parseInt(_tokens[1])*1000);
				} catch (NumberFormatException e) {
					log("Syntax error, wrong number format!");
					e.printStackTrace();
					testComplete(false);
				}
				log("...done waiting.");
	    		testComplete(true);
			} else if (_strLine.startsWith("REM")){
				log(_strLine.replace("REM", "").trim());
	    		testComplete(true);
			} else {
				log ("Syntax error in test file! Maybe an empty line...");
				testComplete(false);
			}
			log("end startTest()");
		}
	
		private String poseComparisonTask(ReasonerConcept _concept1, ReasonerConcept _concept2) {
			String _c1DataID = newDataID();
			try {
				addToWorkingMemory(_c1DataID, 
						new Concept(_concept1.getName(),new ParentFeature()));
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}
//			String _c1fpDataID = newDataID();
//			addToWorkingMemory(_c1fpDataID,
//			new FeaturePointer("Concept",_c1DataID,"")
//			);

			String _c2DataID = newDataID();
			try {
				addToWorkingMemory(_c2DataID, 
						new Concept(_concept2.getName(),new ParentFeature()));
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}
//			String _c2fpDataID = newDataID();
//			addToWorkingMemory(_c2fpDataID,
//			new FeaturePointer("Concept",_c2DataID,"")
//			);

			String _ftCompID = newDataID();
			try {
				addChangeFilter(ChangeFilterFactory.createAddressFilter(
						_ftCompID, m_subarchitectureID, WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleComparisonResult(_wmc);}}
				);
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}
			log("added change filter for the feature comparison task with ID :" + _ftCompID);
			try {
				addToWorkingMemory(_ftCompID,
						new FeatureComparison(
								new FeaturePointer(CASTUtils.typeName(Concept.class),_c1DataID,""),
								"",
								new FeaturePointer(CASTUtils.typeName(Concept.class),_c2DataID,""),
								TriBool.triIndeterminate,
								m_subarchitectureID,
								true));
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}

			String _ftCompTaskID = newDataID();
			try {
				addToWorkingMemory(_ftCompTaskID,
						new FeatureComparisonTask(_ftCompID, m_subarchitectureID));
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}

			return _ftCompID;
		}


		private String poseComparisonTask(AreaID _areaID, ReasonerConcept _concept) {
			String _areaDataID = newDataID();
			try {
				addToWorkingMemory(_areaDataID, _areaID);
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}

			String _conDataID = newDataID();
			try {
				addToWorkingMemory(_conDataID, 
						new Concept(_concept.getName(),new ParentFeature()));
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}

			String _ftCompID = newDataID();
			try {
				addChangeFilter(ChangeFilterFactory.createAddressFilter(
						_ftCompID, m_subarchitectureID, WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleComparisonResult(_wmc);}}
				);
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}
			log("added change filter for the feature comparison task with ID :" + _ftCompID);
			try {
				addToWorkingMemory(_ftCompID,
						new FeatureComparison(
								new FeaturePointer(CASTUtils.typeName(AreaID.class),_areaDataID,""),
								"",
								new FeaturePointer(CASTUtils.typeName(Concept.class),_conDataID,""),
								TriBool.triIndeterminate,
								m_subarchitectureID,
								true));
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}

			String _ftCompTaskID = newDataID();
			try {
				addToWorkingMemory(_ftCompTaskID,
						new FeatureComparisonTask(_ftCompID, m_subarchitectureID));
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			} catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				throw new RuntimeException("Error. Aborting!");
			}

			return _ftCompID;
		}

		
		public void handleComparisonResult(WorkingMemoryChange _wmc) {
			log("got a callback for an overwritten FeatureComparison");
			boolean _complete = false;
	    	try {
	    		// get the WME-ID of the task
	    		String dataId  = _wmc.m_address.m_id;
	    		String subArchId = _wmc.m_address.m_subarchitecture;
	
	    		// get the task from working memory
	    		CASTData data = getWorkingMemoryEntry(dataId, subArchId);
	    		FeatureComparison _comparison = (FeatureComparison) data.getData();
	    		TriBool _boolResult = _comparison.m_featuresEquivalent;
	    		
    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete = true;
    			
    			log(getWorkingMemoryEntry(_comparison.m_proxyFeature.m_address).getClass());
    			log(_comparison.m_proxyFeature.getClass());

    			log("not deleting feature comparison to not break assumptions in the comparators!");
//    			deleteFromWorkingMemory(_comparison.m_proxyFeature.m_address);
//    			deleteFromWorkingMemory(_comparison.m_unionFeature.m_address);
    			    			
//    			String _ft1ID = (String) getWorkingMemoryEntry(_comparison.m_proxyFeature.m_address).getData();
//    			deleteFromWorkingMemory(_ft1ID);
//    			
//    			String _ft2ID = (String) getWorkingMemoryEntry(_comparison.m_unionFeature.m_address).getData();
//    			deleteFromWorkingMemory(_ft2ID);

//    			FeaturePointer _ft2ID = (FeaturePointer) getWorkingMemoryEntry(_comparison.m_unionFeature.m_address).getData();
//    			deleteFromWorkingMemory(_ft2ID.m_address);
    			
    			// not deleting the original FeatureComparison
//    			deleteFromWorkingMemory(dataId);
	    	} catch (SubarchitectureProcessException e) {
	    		log(e.getStackTrace());
	    		_complete = false;
	    	}
	    	testComplete(_complete);
		}
		
		public void handleComaReasonerFunctionChanged(WorkingMemoryChange _wmc) {
		    	log("got a callback: the comaReasoner has evaluated a function!");
			boolean _complete = false;
		    	try {
		    		// get the WME-ID of the function
		    		String dataId  = _wmc.m_address.m_id;
		    		String subArchId = _wmc.m_address.m_subarchitecture;
		
		    		// get the actual function from working memory
		    		CASTData data;
		    		data = getWorkingMemoryEntry(dataId, subArchId);
		    		ComaReasonerFunction _comaRsnrFn = (ComaReasonerFunction) data.getData();
		
		    		TriBool _boolResult;
		    		ComaInstance fnArgIns;
		    		
		    		// specify behavior for certain function types
		    		switch (_comaRsnrFn.m_functiontype.value()) {
		    		case ComaReasonerFunctionType._AreConsEquivalent:
		    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete=(true);
		    			else { 
		    				log("fail: ComaReasoner returned for AreConsEquivalent: "+ ComaHelper.triBool2String(_boolResult));
		    				_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._CompareCons:
		    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete=(true);
		    			else { 
		    				log("fail: ComaReasoner returned for ConpareCons: "+ ComaHelper.triBool2String(_boolResult));
		    				_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._IsConSubcon:
		    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete=(true);
		    			else { 
		    				log("fail: ComaReasoner returned for IsConSubCon: "+ ComaHelper.triBool2String(_boolResult));
		    				_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._IsConSupercon:
		    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete=(true);
		    			else { 
		    				log("fail: ComaReasoner returned for IsConSuperCon: "+ ComaHelper.triBool2String(_boolResult));
		    				_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._GetAllConcepts:
		    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
		    			ComaConcept[] allCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
		    			if (Integer.parseInt(m_reqToResultMap.get(dataId))==allCons.length) _complete=(true);
		    			else {
		    				String _log = "fail: ComaReasoner returned a set of "+allCons.length +" instead of "+m_reqToResultMap.get(dataId)+" concepts for GetAllConcepts for "+fnArgIns.m_name;
			    			for (ComaConcept _concept : allCons) {
			    				_log+= " * "+_concept.m_namespace+_concept.m_sep+_concept.m_name;
			    			}
			    			log(_log);
			    			_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._GetAllDirectConcepts:
		    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
		    			ComaConcept[] allDirectCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
		    			if (Integer.parseInt(m_reqToResultMap.get(dataId))==(allDirectCons.length)) _complete=(true);
		    			else {
		    				String _log = ("fail: ComaReasoner returned a set of "+allDirectCons.length +" instead of "+m_reqToResultMap.get(dataId)+" concepts for GetAllDirectConcepts for "+fnArgIns.m_name);
			    			for (ComaConcept _concept : allDirectCons) {
			    				_log+=" * "+(_concept.m_namespace+_concept.m_sep+_concept.m_name);
			    			}
			    			log(_log);
			    			_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._GetMostSpecificConcepts:
		    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
		    			ComaConcept[] mostSpecCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
		    			if (Integer.parseInt(m_reqToResultMap.get(dataId))==(mostSpecCons.length)) _complete=(true);
		    			else {
		    				String _log=("fail: ComaReasoner returned a set of "+mostSpecCons.length +" instead of "+m_reqToResultMap.get(dataId)+" concepts for GetMostSpecificConcepts for "+fnArgIns.m_name);
			    			for (ComaConcept _concept : mostSpecCons) {
			    				_log+=" * "+(_concept.m_namespace+_concept.m_sep+_concept.m_name);
			    			}
			    			log(_log);
			    			_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._GetAllInstances:
		    			ComaConcept _fnArgCon = ((ComaConcept) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
		    			WorkingMemoryPointer[] allIns = ((WorkingMemoryPointer[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
		    			if (Integer.parseInt(m_reqToResultMap.get(dataId))==(allIns.length)) _complete=(true);
		    			else {
		    				String _log=("fail: ComaReasoner returned a set of "+allIns.length+" instead of "+m_reqToResultMap.get(dataId)+" instances for GetAllInstances for "+_fnArgCon.m_name);
			    			log(_log);
			    			_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._GetRelatedInstances:
		    			// TODO check for polymorphic uses!!!!
		    			ComaInstance _fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
		    			WorkingMemoryPointer[] _relIns = ((WorkingMemoryPointer[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
		    			if (!_comaRsnrFn.m_add_info_ptr.m_address.m_id.equals("")) {
		    				ComaRelation _fnArgRel = ((ComaRelation) getWorkingMemoryEntry(_comaRsnrFn.m_add_info_ptr.m_address).getData());
		    				if (Integer.parseInt(m_reqToResultMap.get(dataId))==(_relIns.length)) _complete=(true);
		        			else {
		        				String _log=("fail: ComaReasoner returned a set of "+_relIns.length+" instead of "+m_reqToResultMap.get(dataId)+" instances for GetRelatedInstances for "+_fnArgIns.m_name +" and "+_fnArgRel.m_name);
		    	    			log(_log);
		    	    			_complete=(false);
		        			}
		    			}
		    			else {
		    				if (Integer.parseInt(m_reqToResultMap.get(dataId))==(_relIns.length)) _complete=(true);
		        			else {
		        				String _log=("fail: ComaReasoner returned a set of "+_relIns.length+" instead of "+m_reqToResultMap.get(dataId)+" instances for GetRelatedInstances for "+_fnArgIns.m_name);
		        				log(_log);
		        				_complete=(false);
		        			}
		    			}
		    			break;
		    		case ComaReasonerFunctionType._GetBasicLevelConcepts:
		    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
		    			ComaConcept[] basicLvlCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
		    			if (Integer.parseInt(m_reqToResultMap.get(dataId))==(basicLvlCons.length)) _complete=(true);
		    			else {
		    				String _log=("fail: ComaReasoner returned a set of "+basicLvlCons.length +" instead of "+m_reqToResultMap.get(dataId)+" concepts for BasicLevelConcepts for "+fnArgIns.m_name);
			    			for (ComaConcept _concept : basicLvlCons) {
			    				_log+=(_concept.m_namespace+_concept.m_sep+_concept.m_name);
			    			}
			    			log(_log);
			    			_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._GetObjectMobility:
		    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete=(true);
		    			else { 
		    				log("fail: ComaReasoner returned for getObjectMobility: "+ ComaHelper.triBool2String(_boolResult));
		    				_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._GetTypicalObjects:
		    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
		    			ComaConcept[] typObjCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
		    			if (Integer.parseInt(m_reqToResultMap.get(dataId))==typObjCons.length) _complete=(true);
		    			else {
		    				String _log = "fail: ComaReasoner returned a set of "+typObjCons.length +" instead of "+m_reqToResultMap.get(dataId)+" concepts for GetTypicalObjects for "+fnArgIns.m_name;
			    			for (ComaConcept _concept : typObjCons) {
			    				_log+= " * "+_concept.m_namespace+_concept.m_sep+_concept.m_name;
			    			}
			    			log(_log);
			    			_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._DeleteInstance:
		    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete=(true);
		    			else { 
		    				log("fail: ComaReasoner returned for DeleteInstance: "+ ComaHelper.triBool2String(_boolResult));
		    				_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._AddInstance:
		    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete=(true);
		    			else { 
		    				log("fail: ComaReasoner returned for AddInstance: "+ ComaHelper.triBool2String(_boolResult));
		    				_complete=(false);
		    			}
		    			break;
		    		case ComaReasonerFunctionType._AddRelation:
		    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
		    			if (m_reqToResultMap.get(dataId).equals(ComaHelper.triBool2String(_boolResult))) _complete=(true);
		    			else { 
		    				log("fail: ComaReasoner returned for AddRelation: "+ ComaHelper.triBool2String(_boolResult));
		    				_complete=(false);
		    			}
		    			break;
		    		default:
		    			log("Unknown action type: " + _comaRsnrFn.m_functiontype.value());
		    		_complete=(false);
		    		break;
		    		}
		
		
		    		// hmm we can probably delete temporary entries from WM
		    		m_funcWriter.cleanUpAfterFunctionEvaluated(_wmc.m_address);

		    		log("reporting test " + dataId + " complete:"+_complete);
		    	} catch (SubarchitectureProcessException e) {
		    		e.printStackTrace();
		    		_complete = false;
		    	}
		    testComplete(_complete);	
		    }

	}
}

