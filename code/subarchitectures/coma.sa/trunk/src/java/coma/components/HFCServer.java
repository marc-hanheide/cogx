package coma.components;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

import Ice.Current;
import Ice.Identity;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;

import comadata.HFCInterface;
import comadata.QueryResults;
import de.dfki.lt.hfc.QueryParseException;

public class HFCServer extends ManagedComponent {
	
	Identity id;
	
	/**
	 * Dunno whether this is still necessary in CAST2... 
	 */
	public HFCServer() {
		super();
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	}
	
	protected void configure(Map<String, String> args) {
		super.configure(args);
		log("registering HFCServer...");
		
		
		
		this.registerIceServer(HFCInterface.class, 
				new HFCInterfaceI(
						(args.get("--stdNSFile")!=null ? args.get("--stdNSFile") : ""), 
						(args.get("--stdTupleFile")!=null ? args.get("--stdTupleFile") : ""), 
						(args.get("--stdRuleFile")!=null ? args.get("--stdRuleFile") : ""),
						(args.get("--otherNSFiles")!=null ? args.get("--otherNSFiles").split(",") : new String[0]), 
						(args.get("--otherTupleFiles")!=null ? args.get("--otherTupleFiles").split(",") : new String[0]), 
						(args.get("--otherRuleFiles")!=null ? args.get("--otherRuleFiles").split(",") : new String[0])));
		
		log("registered HFCServer...");
		
//		// establish ICE identity of the ComaReasonerInterface
//		id = new Identity();
//		id.name=this.getComponentID(); // use my component name
//		id.category="ComaReasoner";
//		
//		if (args.containsKey("--crowl-cfg")) {
//			getObjectAdapter().add(new ComaReasonerInterfaceI(args.get("--crowl-cfg"), m_bLogOutput), id);
//	       	// done registering ComaReasonerInterface	
//		} else {
//			try {
//				throw new CASTException("You need to specify a crowl config file using the --crowl-cfg flag! Exiting...");
//			} catch (CASTException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//				System.exit(0);
//			}
//		}
		
       	
	}

	
	/**
	 * HFCInterfaceI implements the functions specified
	 * in the SLICE definition.
	 * 
	 * This is the ICE object that handles requests to the tuple store.
	 * 
	 * @author zender
	 *
	 */public class HFCInterfaceI extends comadata._HFCInterfaceDisp {

		// ICE related ID
		private static final long serialVersionUID = 6809681574925454933L;

		// HFC related members
		//private de.dfki.lt.hfc.Namespace namespaces = null;
		//private de.dfki.lt.hfc.TupleStore tupleStore = null;
		//private de.dfki.lt.hfc.RuleStore ruleStore = null;
		private de.dfki.lt.hfc.ForwardChainer forwardChainer = null;
		private de.dfki.lt.hfc.Query query = null;
		
		public HFCInterfaceI(
				String std_namespaceFile, 
				String std_tupleFile, 
				String std_ruleFile, 
				String[] other_namespaceFiles, 
				String[] other_tupleFiles, 
				String[] other_ruleFiles ) {
						
			//System.out.println("HFCInterfaceI constructor called.");
			//System.out.println("std_namespaceFile " + std_namespaceFile);
			//System.out.println("std_tupleFile " + std_tupleFile);
			//System.out.println("std_ruleFile " + std_ruleFile);
			//System.out.println("other_namespaceFile " + other_namespaceFiles);
			//System.out.println("other_tupleFiles " + other_tupleFiles);
			//System.out.println("other_ruleFiles " + other_ruleFiles);
			
			// read declarations from files and initialize members
			// wrong... new: just load the default files in the FC constructor
			/*
			this.namespaces = new de.dfki.lt.hfc.Namespace(std_namespaceFile);
			System.out.println("init namespaces.");
			
			this.tupleStore = new de.dfki.lt.hfc.TupleStore(this.namespaces, std_tupleFile);
			System.out.println("init tuplestore.");

			
			this.ruleStore = new de.dfki.lt.hfc.RuleStore(this.namespaces, this.tupleStore, std_ruleFile);
			System.out.println("init rules.");

			//de.dfki.lt.hfc.Interactive intMode = new de.dfki.lt.hfc.Interactive(namespaceFile, tupleFile, ruleFile);
			*/ 
			
			this.forwardChainer = new de.dfki.lt.hfc.ForwardChainer(std_tupleFile, std_ruleFile, std_namespaceFile);
			log("created forward chainer object.");
			
			// so?
			for (String i_tupleFile : other_tupleFiles) {
				this.forwardChainer.uploadTuples(i_tupleFile);
				log("added extra tuples from file " + i_tupleFile);
			}

			// so?
			for (String i_ruleFile : other_ruleFiles) {
				this.forwardChainer.uploadRules(i_ruleFile);
				log("added extra rules from file " + i_ruleFile);
			}

			for (String i_namespaceFile : other_namespaceFiles) {
				this.forwardChainer.namespace.readNamespaces(i_namespaceFile);
				log("added extra namespace file " + i_namespaceFile);
			}

			this.forwardChainer.computeClosure();
			log("Constructed a new HFCInterface!");
			
			this.query = new de.dfki.lt.hfc.Query(this.forwardChainer.tupleStore);
			System.out.println("init query object.");
		}
		
		
		@Override
		public QueryResults querySelect(String q, Current current) {
			log("querySelect() called with query " + q);
			QueryResults _results = new QueryResults();
			
			if (q.toUpperCase().startsWith("SELECT")) {
				log("received a SELECT type query");
				de.dfki.lt.hfc.BindingTable bt;
				_results.query = q;
				
				try {
					log("try...");
					bt = this.query.query(q);
					if (bt == null) {
						log("HFC: query contains constants not known to the tuple store");
						_results.bt = new String[0][0];
						_results.varPosMap = new HashMap<String, Integer>();
					} 
					else {
						log("bt!=null");
						ArrayList<Map<String, String>> _hfcAnswer = 
							de.dfki.lt.hfc.BindingTableDecoder.decode(bt, this.forwardChainer.tupleStore);
						log("received a decoded collection of size: " + _hfcAnswer.size());
						Map<String,String> _firstLine = _hfcAnswer.remove(0);
						_results.bt = new String[_hfcAnswer.size()+1][_firstLine.size()];
						_results.varPosMap = new HashMap<String, Integer>();
						int _posIndex = 0;
						for (String _var : _firstLine.keySet()) {
							_results.varPosMap.put(_var, _posIndex);
							_results.bt[0][_posIndex++] = _firstLine.get(_var);
						}
						
						int _currLineIndex = 1;
						for (Map<String,String> _currLine : _hfcAnswer) {
							for (String _var : _currLine.keySet()) {
								_results.bt[_currLineIndex][_results.varPosMap.get(_var)] = 
									_currLine.get(_var);
							}
							_currLineIndex++;
						}
					}
				} catch (QueryParseException e) {
					log("HFC: malformed SELECT query!");
					_results.bt = new String[0][0];
					_results.varPosMap = new HashMap<String, Integer>();
					e.printStackTrace();
				}
			}
			else {
				log("HFC: malformed SELECT query!");
				_results.bt = new String[0][0];
				_results.varPosMap = new HashMap<String, Integer>();
			}
			
			return _results;
		}


		@Override
		public String ping(Current current) {
			return "pong.";
		}
		
		
		
	}
	
}
