package coma.components;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Properties;
import java.util.TreeMap;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.ContentPlanningGoal;
import org.cognitivesystems.reasoner.base.OntologyMemberFactory;
import org.cognitivesystems.reasoner.base.ReasonerConcept;
import org.cognitivesystems.reasoner.base.ReasonerInstance;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.utils.LFUtils;

import com.hp.hpl.jena.sparql.algebra.Op;

import binding.util.BindingUtils;

import BindingData.BindingProxy;
import BindingFeatures.AreaID;
import BindingFeatures.Concept;
import ComaData.ComaConcept;
import ComaData.ComaInstance;
import ComaData.ComaReasonerFunction;
import ComaData.ComaReasonerFunctionType;
import ComaData.ComaRelation;
import ComaData.ResolveEntity;
import ComaData.StringWrapper;
import ComaData.TriBoolResult;
import TestingData.TestingCommand;
import TestingData.TestingCommandStatus;
import coma.aux.ComaFunctionWriter;
import coma.aux.ComaHelper;
import coma.aux.InstanceUnknownException;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import cast.core.data.CASTData;


/**
 * This class can generate "fake events" inside coma.sa.
 * This means that it can trigger the creation and deletion of
 * instances, and the creation of relation.
 * And it can also execute all possible queries to the coma reasoner,
 * like querying the concept hierarchy (TBox queries), or retrieving
 * instance information (ABox queries).
 * 
 * The "fake events" are triggered either using a dedicated text file
 * (eventfile), or by typing them into a text input window, or finally
 * by being distributed through the Multi-SA testing machinery (MSAT).
 * 
 * If an eventfile is specified as a parameter, the component will
 * go through the file, line by line, executing the listed "events".
 * If no eventfile is given, the component will pop up a text input
 * window that takes "events" using the same syntax.
 * The text input window can be avoided using the --no-gui flag.
 * The component will also in either case listen for TestingCommand
 * structs on its own WM. Whenever a new TestingCommand WME is found,
 * it will execute the specified command. When the result is received,
 * it will confirm success in the original TestingCommand struct.
 * In case of a failure, the component will throw a RuntimeException.
 * 
 * You can type "help" to get instructions about the "event syntax".
 * 
 * Usage:
 *	assertInstance,<ins>,<con>
 *	assertRelation,<ins1>,<ins2>,<rel>
 *  assertNumberTag,<ins>,<number>
 *  assertName,<ins>,<name>
 *	deleteInstance,<ins>
 *	getConcepts,<ins>,[<restrictor>],[<expected cardinality>]
 *	getInstances[,<con>][,<expected cardinality>]
 *	getRelatedInstances,<ins>[,<rel>][,<expected cardinality>]
 *	areConceptsEqui,<con1>,<con2>[,<expected TriBool result>]
 *	isSubconceptOf,<con1>,<con2>[,<expected TriBool result>]
 *	isSuperConceptOf,<con1>,<con2>[,<expected TriBool result>]
 *	areConceptsComparable,<con1>,<con2>[,<expected TriBool result>]
 *	isMobileObject,<con>[,<expected TriBool result>]
 *	getTypicalObjectConcepts,<ins>[,<expected cardinality>]
 *	testRefEx,<ins1:intended referent>,<ins2:origin>
 *	generateRefEx,<concept-of-referent-proxy>
 *	resolveProxy,<concept-of-head-proxy>
 *
 * 
 * @author zender
 *
 */
public class ComaFakeEventGenerator extends PrivilegedManagedProcess {

	// ontology/reasoning related members
	private String m_ontologyNamespace;
	private String m_ontologySep;
	private OntologyMemberFactory m_oeMemberMaker;

	// coma SA-internal interaction
	private ComaFunctionWriter m_funcWriter;
	private HashMap<String, String> m_wmID2query;
	
	// handling answers if query received by multi SA tester
	private TreeMap<String,String> m_comaReasonerFunctionToResultMap;
	private TreeMap<String,WorkingMemoryAddress> m_comaReasonerFunctionToTestingCommand;

	// optional SA IDs 
	private String m_bindingSA;
	private String m_comsysSA;
	
	
	// optional eventfile
	private String m_eventfile;
	// optional text input window
	private boolean m_showGUI;

	// optional GUI stuff
	static boolean OKBUTTON_PUSHED = false ;
    private JTextField textField;
    private JButton ok;

	public ComaFakeEventGenerator(String _id) {
		super(_id);
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		m_wmID2query = new HashMap<String, String>();
		m_comaReasonerFunctionToResultMap = new TreeMap<String, String>();
		m_comaReasonerFunctionToTestingCommand = new TreeMap<String, WorkingMemoryAddress>();
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
            this.m_ontologySep = "oe";
        }
		log("Using namespace: " + m_ontologyNamespace);
		log("Using separator: " + m_ontologySep);

		if (_config.containsKey("--no-gui")) {
			this.m_showGUI = false;
			log("Not showing the text input window.");
		} else {
			this.m_showGUI = true;
		}

		if (_config.containsKey("--eventfile")) {
			this.m_eventfile= _config.getProperty("--eventfile");
			log("Using eventfile: " + m_eventfile);
			if (m_showGUI) log("And not showing the text input window.");
			this.m_showGUI = false;
		} else {
			log("No eventfile specified...");
			if (m_showGUI) log ("Using the text input window.");
		}
		
		if (_config.containsKey("--bsa")) {
			this.m_bindingSA = _config.getProperty("--bsa");
		} else if (_config.containsKey("-bsa")) {
			this.m_bindingSA = _config.getProperty("-bsa");
		}
		
		if (_config.containsKey("--comsys")) {
			this.m_comsysSA = _config.getProperty("--comsys");
		} else if (_config.containsKey("-comsys")) {
			this.m_comsysSA = _config.getProperty("-comsys");
		}

		this.m_oeMemberMaker = new OntologyMemberFactory(m_ontologyNamespace, m_ontologySep);
		this.m_funcWriter = new ComaFunctionWriter(this);

	}

	public void start() {
		super.start();
		// register change event listener for the multi SA tester
		// in case no eventfile of my own is present
		if (m_eventfile==null) {
        	try {
				addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(TestingCommand.class, 
						WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
				        public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				        	log("I got a new testing command!");
				        	processNewTestingCommand(_wmc);
				        }
				    });
			} catch (SubarchitectureProcessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
    }	
	
	protected void runComponent() {
		if (m_eventfile!=null) {
			FileInputStream fstream;
			try {
				fstream = new FileInputStream(m_eventfile);
				// Get the object of DataInputStream
				DataInputStream in = new DataInputStream(fstream);
				BufferedReader br = new BufferedReader(new InputStreamReader(in));
				String strLine;
				//Read File Line By Line
				while ((strLine = br.readLine()) != null)   {
					parseAndExecute(strLine,null);
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
		if (m_showGUI) {
			Frame frame = null;

			ok = new JButton("OK");
			JLabel l = new JLabel("Please enter a COMA command (comma-sep): ");
			textField = new JTextField();
			textField.setPreferredSize(new Dimension(100,20));

			JPanel panel = new JPanel();
			panel.add(ok);
			JDialog dialog = new JDialog(frame, "Coma Event Input");
			dialog.setLocation(600, 600);
			dialog.getContentPane().add(l, BorderLayout.NORTH);
			dialog.getContentPane().add(textField, BorderLayout.CENTER);
			dialog.getContentPane().add(panel, BorderLayout.SOUTH);
			dialog.setSize(300,100);
			dialog.setVisible(true);
			textField.requestFocusInWindow();

			KeyboardFocusManager.getCurrentKeyboardFocusManager()
			.addKeyEventDispatcher(new KeyEventDispatcher(){
				public boolean dispatchKeyEvent(KeyEvent ke){
					if(ke.getID() == KeyEvent.KEY_PRESSED)
					{
						if(((KeyEvent) ke).getKeyCode() == KeyEvent.VK_ENTER)
						{
							ok.doClick();
						}
					}
					return false;
				}
			});

			ok.addActionListener(new ActionListener(){
				public void actionPerformed(ActionEvent e){
					ComaFakeEventGenerator.OKBUTTON_PUSHED = true;
				}
			});

			while (true) {
				while (!OKBUTTON_PUSHED) {
					this.sleepProcess(100);
				}

				ComaFakeEventGenerator.OKBUTTON_PUSHED = false;
				String _inputText = textField.getText();
				if (_inputText.toLowerCase().startsWith("bye")) {
					log("EXITING! Good bye!");
					return;
				}
				parseAndExecute(_inputText,null);
				textField.setText("");
			}
		}
	}
	
	private void processNewTestingCommand(WorkingMemoryChange _wmc) {
        CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
        TestingCommand _currCmd = (TestingCommand) wme.getData();
        
        m_comaReasonerFunctionToTestingCommand.put(
        		parseAndExecute(_currCmd.m_command, 
				new WorkingMemoryChangeReceiver() {
			        public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			        	log("I got a result for a testing command!");
			        	processTestingCommandResult(_wmc);
			        }}),
			        _wmc.m_address);
        }
	
	private void processTestingCommandResult(WorkingMemoryChange _wmc) {
		CASTData<?> wme;
		try {
			wme = getWorkingMemoryEntry(_wmc.m_address);

			String _expectedResult = m_comaReasonerFunctionToResultMap.remove(_wmc.m_address.m_id);
			WorkingMemoryAddress _originalTestingCmd = m_comaReasonerFunctionToTestingCommand.remove(_wmc.m_address.m_id);

			// remember to check if we actually wanted to get any result at all!
			// get the WME-ID of the function
			String dataId  = _wmc.m_address.m_id;

			log("The original query was: " + m_wmID2query.remove(dataId));

			ComaReasonerFunction _comaRsnrFn = (ComaReasonerFunction) wme.getData();

			TriBool _boolResult;
			ComaInstance fnArgIns;

			// specify behavior for certain function types
			switch (_comaRsnrFn.m_functiontype.value()) {
			case ComaReasonerFunctionType._AreConsEquivalent:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for AreConsEquivalent: "+ ComaHelper.triBool2String(_boolResult));
				if (_expectedResult!=null && !ComaHelper.triBool2String(_boolResult).equals(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._CompareCons:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for ConpareCons: "+ ComaHelper.triBool2String(_boolResult));
				if (_expectedResult!=null && !ComaHelper.triBool2String(_boolResult).equals(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._IsConSubcon:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for IsConSubCon: "+ ComaHelper.triBool2String(_boolResult));
				if (_expectedResult!=null && !ComaHelper.triBool2String(_boolResult).equals(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._IsConSupercon:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for IsConSuperCon: "+ ComaHelper.triBool2String(_boolResult));
				if (_expectedResult!=null && !ComaHelper.triBool2String(_boolResult).equals(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._GetAllConcepts:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] allCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				String _log = "ComaReasoner returned a set of "+allCons.length +" concepts for GetAllConcepts for "+fnArgIns.m_name;
				for (ComaConcept _concept : allCons) {
					_log+= " * "+_concept.m_namespace+_concept.m_sep+_concept.m_name;
				}
				log(_log);
				if (_expectedResult!=null && allCons.length!=Integer.parseInt(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._GetAllDirectConcepts:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] allDirectCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				String _log1 = ("ComaReasoner returned a set of "+allDirectCons.length +" concepts for GetAllDirectConcepts for "+fnArgIns.m_name);
				for (ComaConcept _concept : allDirectCons) {
					_log1+=" * "+(_concept.m_namespace+_concept.m_sep+_concept.m_name);
				}
				log(_log1);
				if (_expectedResult!=null && allDirectCons.length!=Integer.parseInt(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._GetMostSpecificConcepts:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] mostSpecCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				String _log2=("ComaReasoner returned a set of "+mostSpecCons.length +" concepts for GetMostSpecificConcepts for "+fnArgIns.m_name);
				for (ComaConcept _concept : mostSpecCons) {
					_log2+=" * "+(_concept.m_namespace+_concept.m_sep+_concept.m_name);
				}
				log(_log2);
				if (_expectedResult!=null && mostSpecCons.length!=Integer.parseInt(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._GetAllInstances:
				log("reading result of GetAllInstances:");
				ComaConcept _fnArgCon = (ComaConcept) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData();

				CASTData<?> _wme_ = getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address);
				log("WME Type = "+_wme_.getType());
				WorkingMemoryPointer[] allInsPtrs = (WorkingMemoryPointer[]) _wme_.getData();

				String _log3=("ComaReasoner returned a set of "+allInsPtrs.length+" instances for GetAllInstances for "+_fnArgCon.m_name);
				for (WorkingMemoryPointer _insPtr: allInsPtrs) {
					ComaInstance _ins = (ComaInstance) getWorkingMemoryEntry(_insPtr.m_address).getData();
					_log3+=" * "+(_ins.m_namespace+_ins.m_sep+_ins.m_name);
				}     		
				log(_log3);
				if (_expectedResult!=null && allInsPtrs.length!=Integer.parseInt(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._GetRelatedInstances:
				// TODO check for polymorphic uses!!!!
				ComaInstance _fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				WorkingMemoryPointer[] _relInsPtrs = ((WorkingMemoryPointer[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				if (!_comaRsnrFn.m_add_info_ptr.m_address.m_id.equals("")) {
					ComaRelation _fnArgRel = ((ComaRelation) getWorkingMemoryEntry(_comaRsnrFn.m_add_info_ptr.m_address).getData());
					String _log4=("ComaReasoner returned a set of "+_relInsPtrs.length+" instances for GetRelatedInstances for "+_fnArgIns.m_name +" and "+_fnArgRel.m_name);
					for (WorkingMemoryPointer _rnsPtr: _relInsPtrs) {
						ComaInstance _ins = (ComaInstance) getWorkingMemoryEntry(_rnsPtr.m_address).getData();
						_log4+=" * "+(_ins.m_namespace+_ins.m_sep+_ins.m_name);
					}     		
					log(_log4);
				}
				else {
					String _log5=("ComaReasoner returned a set of "+_relInsPtrs.length+" instances for GetRelatedInstances for "+_fnArgIns.m_name);
					for (WorkingMemoryPointer _rnsPtr: _relInsPtrs) {
						ComaInstance _ins = (ComaInstance) getWorkingMemoryEntry(_rnsPtr.m_address).getData();
						_log5+=" * "+(_ins.m_namespace+_ins.m_sep+_ins.m_name);
					}     		
					log(_log5);
				}
				if (_expectedResult!=null && _relInsPtrs.length!=Integer.parseInt(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._GetBasicLevelConcepts:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] basicLvlCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				String _log6=("fail: ComaReasoner returned a set of "+basicLvlCons.length +" concepts for BasicLevelConcepts for "+fnArgIns.m_name);
				for (ComaConcept _concept : basicLvlCons) {
					_log6+=(_concept.m_namespace+_concept.m_sep+_concept.m_name);
				}
				log(_log6);
				if (_expectedResult!=null && basicLvlCons.length!=Integer.parseInt(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._GetObjectMobility:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for getObjectMobility: "+ ComaHelper.triBool2String(_boolResult));
				if (_expectedResult!=null && !ComaHelper.triBool2String(_boolResult).equals(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._GetTypicalObjects:
				fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
				ComaConcept[] typObjCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
				String _log7 = "fail: ComaReasoner returned a set of "+typObjCons.length +" concepts for GetTypicalObjects for "+fnArgIns.m_name;
				for (ComaConcept _concept : typObjCons) {
					_log7+= " * "+_concept.m_namespace+_concept.m_sep+_concept.m_name;
				}
				log(_log7);
				if (_expectedResult!=null && typObjCons.length!=Integer.parseInt(_expectedResult)) {
					throw new RuntimeException("result did not match expected result " +
							_expectedResult);
				}
				break;
			case ComaReasonerFunctionType._DeleteInstance:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for DeleteInstance: "+ ComaHelper.triBool2String(_boolResult));
				break;
			case ComaReasonerFunctionType._AddInstance:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for AddInstance: "+ ComaHelper.triBool2String(_boolResult));
				break;
			case ComaReasonerFunctionType._AddRelation:
				_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
				log("ComaReasoner returned for AddRelation: "+ ComaHelper.triBool2String(_boolResult));
				break;
			default:
				log("Unknown action type: " + _comaRsnrFn.m_functiontype.value());
			break;
			}

			// we should now write back the status COMPLETED 
			// to the original testing command struct
			// if the expected result did not match, a RunTimeException
			// has already been thrown anyway, and we wouldn't have ended up here...
			TestingCommand _originalCmd = (TestingCommand) getWorkingMemoryEntry(_originalTestingCmd).getData();
			_originalCmd.m_status = TestingCommandStatus.COMPLETED;
			overwriteWorkingMemory(_originalTestingCmd, _originalCmd);
			log("wrote back COMPLETED to the original TestingCommand.");

			// hmm we can probably delete temporary entries from WM
			m_funcWriter.cleanUpAfterFunctionEvaluated(_wmc.m_address);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException(e);
		}
	}

	private String parseAndExecute(String _strLine, WorkingMemoryChangeReceiver _WMCReceiver) {
		String _returnFuncID = "";
		if (_WMCReceiver==null) {
			_WMCReceiver = new WorkingMemoryChangeReceiver() {
			  	public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			  		handleComaReasonerFunctionChanged(_wmc);} };
		}
		try {
    	if (_strLine.startsWith("instance") || _strLine.startsWith("assertInstance")) {
			String[] _tokens = _strLine.split(",");
			for (int i = 0; i < _tokens.length; i++) {
				log(_tokens[i]);					
			}
			ReasonerInstance _ins = m_oeMemberMaker.createInstance(_tokens[1]);
			ReasonerConcept _con = m_oeMemberMaker.createConcept(_tokens[2]);
			_returnFuncID = m_funcWriter.addInstanceConceptAssertion(_WMCReceiver,_ins, _con);
		} else if (_strLine.startsWith("relation") || _strLine.startsWith("assertRelation")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.addInstanceInstanceRelation(_WMCReceiver, 
					m_oeMemberMaker.createInstance(_tokens[1]), m_oeMemberMaker.createInstance(_tokens[2]), m_oeMemberMaker.createRelation(_tokens[3]));
		} else if (_strLine.startsWith("assertName")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.addInstanceName(_WMCReceiver, 
					m_oeMemberMaker.createInstance(_tokens[1]), _tokens[2]);
		} else if (_strLine.startsWith("assertNumberTag")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.addInstanceNumberTag(_WMCReceiver, 
					m_oeMemberMaker.createInstance(_tokens[1]), Integer.parseInt(_tokens[2]));
		} else if (_strLine.startsWith("deleteins") || _strLine.startsWith("deleteInstance")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.deleteInstance(_WMCReceiver,
					m_oeMemberMaker.createInstance(_tokens[1]));
		} else if (_strLine.startsWith("wait")){
			String[] _tokens = _strLine.split(",");
			log("Waiting for "+_tokens[1]+"seconds...");
			try {
				sleepProcess(Integer.parseInt(_tokens[1])*1000);
			} catch (NumberFormatException e) {
				log("Syntax error, wrong number format!");
				e.printStackTrace();
				throw new RuntimeException(e);
			}
			log("...done waiting.");
		} else if (_strLine.startsWith("getcons")) {
			String[] _tokens = _strLine.split(",");
			ComaFunctionWriter.ConceptQueryRestrictor _res;
			if (_tokens[2].equals("ALL")) _res=ComaFunctionWriter.ConceptQueryRestrictor.ALL; 
			else if (_tokens[2].equals("MOSTSPECIFIC")) _res=ComaFunctionWriter.ConceptQueryRestrictor.MOSTSPECIFIC; 
			else if (_tokens[2].equals("DIRECT")) _res=ComaFunctionWriter.ConceptQueryRestrictor.DIRECT;
			else if (_tokens[2].equals("BASICLEVEL")) _res=ComaFunctionWriter.ConceptQueryRestrictor.BASICLEVEL;
			else _res=ComaFunctionWriter.ConceptQueryRestrictor.ALL;
			try {
				_returnFuncID = m_funcWriter.getConcepts(_WMCReceiver, 
				  		m_oeMemberMaker.createInstance(_tokens[1]), _res);
				m_wmID2query.put(_returnFuncID,_strLine);
					
			} catch (InstanceUnknownException e) {
				e.printStackTrace();
//				throw new RuntimeException("Error: instance unkown! Aborting!");
			}
		} else if (_strLine.startsWith("getConcepts")) {
			String[] _tokens = _strLine.split(",");
			ComaFunctionWriter.ConceptQueryRestrictor _res;
			String _expectedResult="";
			if (_tokens.length>2) {
				if (_tokens[2].equals("ALL")) _res=ComaFunctionWriter.ConceptQueryRestrictor.ALL; 
				else if (_tokens[2].equals("MOSTSPECIFIC")) _res=ComaFunctionWriter.ConceptQueryRestrictor.MOSTSPECIFIC; 
				else if (_tokens[2].equals("DIRECT")) _res=ComaFunctionWriter.ConceptQueryRestrictor.DIRECT;
				else if (_tokens[2].equals("BASICLEVEL")) _res=ComaFunctionWriter.ConceptQueryRestrictor.BASICLEVEL;
				else {
					_res=ComaFunctionWriter.ConceptQueryRestrictor.ALL;
					if (_tokens.length==3) _expectedResult=_tokens[2];
				}
				if (_tokens.length==4) _expectedResult=_tokens[3];
			} else {
				_res=ComaFunctionWriter.ConceptQueryRestrictor.ALL;
			}
			try {
				_returnFuncID = m_funcWriter.getConcepts(_WMCReceiver, 
				  		m_oeMemberMaker.createInstance(_tokens[1]), _res);
				m_wmID2query.put(_returnFuncID,_strLine);
				if (!_expectedResult.equals("")) {
					m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
				}
			} catch (InstanceUnknownException e) {
				e.printStackTrace();
//				throw new RuntimeException("Error: instance unkown! Aborting!");
			}
		} else if (_strLine.startsWith("getallins") || _strLine.startsWith("getInstances")) {
			String[] _tokens = _strLine.split(",");
			if (_tokens.length>1) {
				_returnFuncID= m_funcWriter.getAllInstances(_WMCReceiver,
					  		m_oeMemberMaker.createConcept(_tokens[1]));
			} else {
				_returnFuncID= m_funcWriter.getAllInstances(_WMCReceiver,
				  		m_oeMemberMaker.createConcept("owl:Thing"));
			}
			String _expectedResult = "";
			if (_tokens.length>2) _expectedResult=_tokens[2];
			m_wmID2query.put(_returnFuncID,_strLine);
			if (!_expectedResult.equals("")) {
				m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
			}
		} else if (_strLine.startsWith("getallrelins")) {
			String[] _tokens = _strLine.split(",");
			try {
				m_wmID2query.put(m_funcWriter.getRelatedInstances(_WMCReceiver,
						  		m_oeMemberMaker.createInstance(_tokens[1])),_strLine);
			} catch (InstanceUnknownException e) {
				e.printStackTrace();
//				throw new RuntimeException("Error: instance unkown! Aborting!");
			}
		} else if (_strLine.startsWith("getrelins")) {
			String[] _tokens = _strLine.split(",");
			try {
				m_wmID2query.put(m_funcWriter.getRelatedInstances(_WMCReceiver,
						  		m_oeMemberMaker.createInstance(_tokens[1]),
						  		m_oeMemberMaker.createRelation(_tokens[2])),_strLine);
			} catch (InstanceUnknownException e) {
				e.printStackTrace();
//				throw new RuntimeException("Error: instance unkown! Aborting!");
			}
		} else if (_strLine.startsWith("getRelatedInstances")) {
			String[] _tokens = _strLine.split(",");
			if (_tokens.length==2) {
				try {
					_returnFuncID= m_funcWriter.getRelatedInstances(_WMCReceiver,
							  		m_oeMemberMaker.createInstance(_tokens[1]));
					m_wmID2query.put(_returnFuncID,_strLine);
				} catch (InstanceUnknownException e) {
					e.printStackTrace();
//					throw new RuntimeException("Error: instance unkown! Aborting!");
				}				
			} else if (_tokens.length>3) {
				try {
					int _expResInt = Integer.parseInt(_tokens[2]);
					try {
						_returnFuncID = m_funcWriter.getRelatedInstances(_WMCReceiver,
								  		m_oeMemberMaker.createInstance(_tokens[1]));
						String _expectedResult = _tokens[2];
						m_wmID2query.put(_returnFuncID,_strLine);
						m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
					} catch (InstanceUnknownException e) {
						e.printStackTrace();
//						throw new RuntimeException("Error: instance unkown! Aborting!");
					}									
				} catch (NumberFormatException dummyException) {
					try {
						_returnFuncID = m_funcWriter.getRelatedInstances(_WMCReceiver,
								  		m_oeMemberMaker.createInstance(_tokens[1]),
								  		m_oeMemberMaker.createRelation(_tokens[2]));
						String _expectedResult = "";
						if (_tokens.length>3) _expectedResult=_tokens[3];
						m_wmID2query.put(_returnFuncID,_strLine);
						if (!_expectedResult.equals("")) {
							m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
						}
					} catch (InstanceUnknownException e) {
						e.printStackTrace();
//						throw new RuntimeException("Error: instance unkown! Aborting!");
					}
				}
			}				
		} else if (_strLine.startsWith("consequi") || _strLine.startsWith("areConceptsEqui")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.askConsEqui(_WMCReceiver,
					  		m_oeMemberMaker.createConcept(_tokens[1]),
					  		m_oeMemberMaker.createConcept(_tokens[2]));
			String _expectedResult = "";
			if (_tokens.length>3) _expectedResult=_tokens[3];
			m_wmID2query.put(_returnFuncID,_strLine);
			if (!_expectedResult.equals("")) {
				m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
			}
		} else if (_strLine.startsWith("issubcon") || _strLine.startsWith("isSubconceptOf")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.askSubCon(_WMCReceiver,
					  		m_oeMemberMaker.createConcept(_tokens[1]),
					  		m_oeMemberMaker.createConcept(_tokens[2]));
			String _expectedResult = "";
			if (_tokens.length>3) _expectedResult=_tokens[3];
			m_wmID2query.put(_returnFuncID,_strLine);
			if (!_expectedResult.equals("")) {
				m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
			}
		} else if (_strLine.startsWith("issupercon") || _strLine.startsWith("isSuperConceptOf")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.askSuperCon(_WMCReceiver,
					  		m_oeMemberMaker.createConcept(_tokens[1]),
					  		m_oeMemberMaker.createConcept(_tokens[2]));
			String _expectedResult = "";
			if (_tokens.length>3) _expectedResult=_tokens[3];
			m_wmID2query.put(_returnFuncID,_strLine);
			if (!_expectedResult.equals("")) {
				m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
			}
		} else if (_strLine.startsWith("comparable") || _strLine.startsWith("areConceptsComparable")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.compareCons(_WMCReceiver,
					  		m_oeMemberMaker.createConcept(_tokens[1]),
					  		m_oeMemberMaker.createConcept(_tokens[2]),null);
			String _expectedResult = "";
			if (_tokens.length>3) _expectedResult=_tokens[3];
			m_wmID2query.put(_returnFuncID,_strLine);
			if (!_expectedResult.equals("")) {
				m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
			}
		} else if (_strLine.startsWith("mobility") || _strLine.startsWith("isMobileObject")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.getMobility(_WMCReceiver,
					  		m_oeMemberMaker.createConcept(_tokens[1]));
			String _expectedResult = "";
			if (_tokens.length>2) _expectedResult=_tokens[2];
			m_wmID2query.put(_returnFuncID,_strLine);
			if (!_expectedResult.equals("")) {
				m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
			}
		} else if (_strLine.startsWith("typicalobjs") || _strLine.startsWith("getTypicalObjectConcepts")) {
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.getTypicalObjs(_WMCReceiver,
					  		m_oeMemberMaker.createInstance(_tokens[1]));
			String _expectedResult = "";
			if (_tokens.length>2) _expectedResult=_tokens[2];
			m_wmID2query.put(_returnFuncID,_strLine);
			if (!_expectedResult.equals("")) {
				m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
			}			
		} else if (_strLine.startsWith("testRefEx")) {
			log(_strLine);
			String[] _tokens = _strLine.split(",");
			_returnFuncID = m_funcWriter.generateRefEx(_WMCReceiver,
					  		m_oeMemberMaker.createInstance(_tokens[1]),
					  		m_oeMemberMaker.createInstance(_tokens[2]),			  		
							getSubarchitectureID());
			String _expectedResult = "";
			if (_tokens.length>2) _expectedResult=_tokens[2];
			m_wmID2query.put(_returnFuncID,_strLine);
			if (!_expectedResult.equals("")) {
				m_comaReasonerFunctionToResultMap.put(_returnFuncID, _expectedResult);
			}			
		} else if (_strLine.startsWith("generateRefEx")) {
			log(_strLine);
			if (m_bindingSA==null && m_comsysSA==null) {
				log("If you want to generate a ref ex for a proxy, you have to specifiy -bsa or --bsa AND -comsys or --comsys for the FakeEventGenerator.");
			} else {
				String[] _tokens = _strLine.split(",");
				CASTData<BindingProxy>[] _allProxiesD = getWorkingMemoryEntries(m_bindingSA, BindingProxy.class);
				boolean _wroteTask = false;
				for (CASTData<BindingProxy> data : _allProxiesD) {
					if (_wroteTask) break;
					BindingProxy _currProxy = data.getData();
					if (_tokens[1].startsWith("aid:")) {
						log("looking for area ID.");
						ArrayList<AreaID> _areaIDs = BindingUtils.getBindingFeatures(this, m_bindingSA, _currProxy, AreaID.class);
						for (AreaID _aID : _areaIDs) {
							log("found area ID feature with value: " + _aID.m_id);
							if (_wroteTask) break;
							log("new Integer(_aID.m_id).toString() = " + new Integer(_aID.m_id).toString());
							log("_tokens[1].replace(\"aid:\", \"\") = " +_tokens[1].replace("aid:", ""));
							if (new Integer(_aID.m_id).toString().equals(_tokens[1].replace("aid:", ""))) {
								
								log("found a good proxy with area ID=" + _tokens[1].replace("aid:", ""));
								WorkingMemoryAddress _intdRefProxyAdd = new WorkingMemoryAddress(data.getID(),m_bindingSA);

//								LogicalForm _myGRELF = LFUtils.convertFromString(
//										"@d1:dvp(CG ^ <Gre>(" + 
//										_intdRefProxyAdd.m_id.replace(":", "_") + "#" + _intdRefProxyAdd.m_subarchitecture.replace(":", "_") + 
//								":gre ^ bla))");

								LogicalForm _myGRELF = LFUtils.convertFromString(
										"@"+ 
										_intdRefProxyAdd.m_id.replace(":", "_") +
										":rfx()"); 
//								@t1:rfx()
								
								log("my GRE LF looks like this: " + LFUtils.lfToString(_myGRELF));
								
								ContentPlanningGoal _cpGoal = new ContentPlanningGoal("", _myGRELF);
								
								log("going to add ContentPlanningGoal to WM: cpID: " + _cpGoal.cpgid + " -- cpLF:" + LFUtils.lfToString(_myGRELF) );
								
								addToWorkingMemory(newDataID(), m_comsysSA, _cpGoal, OperationMode.BLOCKING);

								log("wrote ContentPlanningGoal struct");
								_wroteTask=true;
								break;
							}
							if (_wroteTask) break;
						}

					
					} else {
						ArrayList<Concept> _concepts = BindingUtils.getBindingFeatures(this, m_bindingSA, _currProxy, Concept.class);
						for (Concept concept : _concepts) {
							if (_wroteTask) break;
							if (concept.m_concept.equals(_tokens[1])) {
								log("found a good proxy with concept=" + _tokens[1]);
								WorkingMemoryAddress _intdRefProxyAdd = new WorkingMemoryAddress(data.getID(),m_bindingSA);

								LogicalForm _myGRELF = LFUtils.convertFromString(
										"@d1:dvp(CG ^ <Gre>(" + 
										_intdRefProxyAdd.toString() + 
								":gre ^ bla))");

								ContentPlanningGoal _cpGoal = new ContentPlanningGoal("", _myGRELF);
								addToWorkingMemory(newDataID(), m_comsysSA, _cpGoal, OperationMode.BLOCKING);

								log("wrote ContentPlanningGoal struct");
								_wroteTask=true;
								break;
							}
						}
					}
				}
				if (!_wroteTask) log("Couldn't find a matching proxy with concept=" + _tokens[1]);
			}
		} else if (_strLine.startsWith("resolveProxy")) {
			log(_strLine);
			if (m_bindingSA==null) {
				log("If you want to resolve a proxy, you have to specifiy -bsa or --bsa for the FakeEventGenerator.");
			} else {
				String[] _tokens = _strLine.split(",");
				CASTData<BindingProxy>[] _allProxiesD = getWorkingMemoryEntries(m_bindingSA, BindingProxy.class);
				boolean _wroteTask = false;
				for (CASTData<BindingProxy> data : _allProxiesD) {
					if (_wroteTask) break;
					BindingProxy _currProxy = data.getData();
					ArrayList<Concept> _concepts = BindingUtils.getBindingFeatures(this, m_bindingSA, _currProxy, Concept.class);
					for (Concept concept : _concepts) {
						if (_wroteTask) break;
						if (concept.m_concept.equals(_tokens[1])) {
							log("found a good proxy with concept=" + _tokens[1]);
							addToWorkingMemory(newDataID(), new ResolveEntity(
									new WorkingMemoryPointer(CASTUtils.typeName(_currProxy),
											new WorkingMemoryAddress(data.getID(),m_bindingSA))));
							log("wrote ResolveEntity struct");
							_wroteTask=true;
							break;
						}
					}
				}
				if (!_wroteTask) log("Couldn't find a matching proxy with concept=" + _tokens[1]);
			}
		} else if (_strLine.startsWith("REM")){
			log(_strLine.replace("REM", "").trim());
		} else if (_strLine.startsWith("#")){
			// comment: do not do anything
		} else {
			if (!_strLine.startsWith("help")){
				log ("Syntax error!");
			}
			log("Usage:");
			log("assertInstance,<ins>,<con>");
			log("assertName,<ins>,<name>");
			log("assertNumberTag,<ins>,<number>");
			log("assertRelation,<ins1>,<ins2>,<rel>");
			log("deleteInstance,<ins>");
			log("getConcepts,<ins>[,<restrictor>][,<expected cardinality>]");
			log("getInstances[,<con>][,<expected cardinality>]");
			log("getRelatedInstances,<ins>[,<rel>][,<expected cardinality>]");
			log("areConceptsEqui,<con1>,<con2>[,<expected TriBool result>]");
			log("isSubconceptOf,<con1>,<con2>[,<expected TriBool result>]");
			log("isSuperConceptOf,<con1>,<con2>[,<expected TriBool result>]");
			log("areConceptsComparable,<con1>,<con2>[,<expected TriBool result>]");
			log("isMobileObject,<con>[,<expected TriBool result>]");
			log("getTypicalObjectConcepts,<ins>[,<expected cardinality>]");
			log("testRefEx,<ins1:intended referent>,<ins2:origin>");
			log("resolveProxy,<concept-of-head-proxy>");
		}
		} catch (ArrayIndexOutOfBoundsException e) {
			log("Syntax error!");
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			log("SA process exception caught... continuing.");
		}
		return _returnFuncID;
	}
    
    
	public void handleComaReasonerFunctionChanged(WorkingMemoryChange _wmc) {
    	log("got a callback: the comaReasoner has evaluated a function!");
    	try {
    		// get the WME-ID of the function
    		String dataId  = _wmc.m_address.m_id;
    		String subArchId = _wmc.m_address.m_subarchitecture;
    		
    		log("The original query was: " + m_wmID2query.remove(dataId));
    		
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
    			log("ComaReasoner returned for AreConsEquivalent: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._CompareCons:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for ConpareCons: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._IsConSubcon:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for IsConSubCon: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._IsConSupercon:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for IsConSuperCon: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._GetAllConcepts:
    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
    			ComaConcept[] allCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
    			String _log = "ComaReasoner returned a set of "+allCons.length +" concepts for GetAllConcepts for "+fnArgIns.m_name;
    			for (ComaConcept _concept : allCons) {
    				_log+= " * "+_concept.m_namespace+_concept.m_sep+_concept.m_name;
    			}
    			log(_log);
    			break;
    		case ComaReasonerFunctionType._GetAllDirectConcepts:
    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
    			ComaConcept[] allDirectCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
    			String _log1 = ("ComaReasoner returned a set of "+allDirectCons.length +" concepts for GetAllDirectConcepts for "+fnArgIns.m_name);
    			for (ComaConcept _concept : allDirectCons) {
    				_log1+=" * "+(_concept.m_namespace+_concept.m_sep+_concept.m_name);
    			}
    			log(_log1);
    			break;
    		case ComaReasonerFunctionType._GetMostSpecificConcepts:
    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
    			ComaConcept[] mostSpecCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
    			String _log2=("ComaReasoner returned a set of "+mostSpecCons.length +" concepts for GetMostSpecificConcepts for "+fnArgIns.m_name);
    			for (ComaConcept _concept : mostSpecCons) {
    				_log2+=" * "+(_concept.m_namespace+_concept.m_sep+_concept.m_name);
    			}
    			log(_log2);
    			break;
    		case ComaReasonerFunctionType._GetAllInstances:
    			log("reading result of GetAllInstances:");
    			ComaConcept _fnArgCon = (ComaConcept) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData();
    			
    			CASTData<?> _wme_ = getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address);
    			log("WME Type = "+_wme_.getType());
    			WorkingMemoryPointer[] allInsPtrs = (WorkingMemoryPointer[]) _wme_.getData();
    			
    			String _log3=("ComaReasoner returned a set of "+allInsPtrs.length+" instances for GetAllInstances for "+_fnArgCon.m_name);
    			for (WorkingMemoryPointer _insPtr: allInsPtrs) {
    				ComaInstance _ins = (ComaInstance) getWorkingMemoryEntry(_insPtr.m_address).getData();
    				_log3+=" * "+(_ins.m_namespace+_ins.m_sep+_ins.m_name);
    			}     		
    			log(_log3);
    			break;
    		case ComaReasonerFunctionType._GetRelatedInstances:
    			// TODO check for polymorphic uses!!!!
    			ComaInstance _fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
    			WorkingMemoryPointer[] _relInsPtrs = ((WorkingMemoryPointer[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
    			if (!_comaRsnrFn.m_add_info_ptr.m_address.m_id.equals("")) {
    				ComaRelation _fnArgRel = ((ComaRelation) getWorkingMemoryEntry(_comaRsnrFn.m_add_info_ptr.m_address).getData());
    				String _log4=("ComaReasoner returned a set of "+_relInsPtrs.length+" instances for GetRelatedInstances for "+_fnArgIns.m_name +" and "+_fnArgRel.m_name);
        			for (WorkingMemoryPointer _rnsPtr: _relInsPtrs) {
        				ComaInstance _ins = (ComaInstance) getWorkingMemoryEntry(_rnsPtr.m_address).getData();
        				_log4+=" * "+(_ins.m_namespace+_ins.m_sep+_ins.m_name);
        			}     		
    				log(_log4);
    			}
    			else {
    				String _log5=("ComaReasoner returned a set of "+_relInsPtrs.length+" instances for GetRelatedInstances for "+_fnArgIns.m_name);
        			for (WorkingMemoryPointer _rnsPtr: _relInsPtrs) {
        				ComaInstance _ins = (ComaInstance) getWorkingMemoryEntry(_rnsPtr.m_address).getData();
        				_log5+=" * "+(_ins.m_namespace+_ins.m_sep+_ins.m_name);
        			}     		
    				log(_log5);
    			}
    			break;
    		case ComaReasonerFunctionType._GetBasicLevelConcepts:
    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
    			ComaConcept[] basicLvlCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
    			String _log6=("fail: ComaReasoner returned a set of "+basicLvlCons.length +" concepts for BasicLevelConcepts for "+fnArgIns.m_name);
    			for (ComaConcept _concept : basicLvlCons) {
    				_log6+=(_concept.m_namespace+_concept.m_sep+_concept.m_name);
    			}
    			log(_log6);
    			break;
    		case ComaReasonerFunctionType._GetObjectMobility:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for getObjectMobility: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._GenerateRefEx:
    			String _refEx = ((StringWrapper) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_string;
    			log("ComaReasoner returned for generateRefEx: "+ _refEx);
    			break;
    		case ComaReasonerFunctionType._GetTypicalObjects:
    			fnArgIns = ((ComaInstance) getWorkingMemoryEntry(_comaRsnrFn.m_arg1ptr.m_address).getData());
    			ComaConcept[] typObjCons = ((ComaConcept[]) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData());
    			String _log7 = "fail: ComaReasoner returned a set of "+typObjCons.length +" concepts for GetTypicalObjects for "+fnArgIns.m_name;
    			for (ComaConcept _concept : typObjCons) {
    				_log7+= " * "+_concept.m_namespace+_concept.m_sep+_concept.m_name;
    			}
    			log(_log7);
    			break;
    		case ComaReasonerFunctionType._DeleteInstance:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for DeleteInstance: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._AddInstance:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for AddInstance: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._AddInstanceName:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for AddInstanceName: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._AddInstanceNumberTag:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for AddInstanceNumberTag: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		case ComaReasonerFunctionType._AddRelation:
    			_boolResult = ((TriBoolResult) getWorkingMemoryEntry(_comaRsnrFn.m_resultptr.m_address).getData()).m_tribool;
    			log("ComaReasoner returned for AddRelation: "+ ComaHelper.triBool2String(_boolResult));
    			break;
    		default:
    			log("Unknown action type: " + _comaRsnrFn.m_functiontype.value());
    		break;
    		}

    		// hmm we can probably delete temporary entries from WM
    		m_funcWriter.cleanUpAfterFunctionEvaluated(_wmc.m_address);

    	} catch (SubarchitectureProcessException e) {
    		e.printStackTrace();
    	}
    }

	protected void taskAdopted(String _taskID) {
		// TODO Auto-generated method stub
	}

	protected void taskRejected(String _taskID) {
		// TODO Auto-generated method stub
	}
}
