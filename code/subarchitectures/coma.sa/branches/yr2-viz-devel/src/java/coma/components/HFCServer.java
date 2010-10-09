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
		log("registering HFCServer...");
		
		this.registerIceServer(HFCInterface.class, new HFCInterfaceI(args.get("--nsFile"), args.get("--tupleFile"), args.get("--ruleFile")));
		
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
		private de.dfki.lt.hfc.Namespace namespaces = null;
		private de.dfki.lt.hfc.TupleStore tupleStore = null;
		private de.dfki.lt.hfc.RuleStore ruleStore = null;
		private de.dfki.lt.hfc.ForwardChainer forwardChainer = null;
		private de.dfki.lt.hfc.Query query = null;
		
		public HFCInterfaceI(String namespaceFile, String tupleFile, String ruleFile) {
			
			System.out.println("HFCInterfaceI constructor called.");
			// read declarations from file and initialize members
			this.namespaces = new de.dfki.lt.hfc.Namespace(namespaceFile);
			System.out.println("loaded namespaces.");
			this.tupleStore = new de.dfki.lt.hfc.TupleStore(this.namespaces, tupleFile);
			System.out.println("loaded tuplestore.");
			this.query = new de.dfki.lt.hfc.Query(this.tupleStore);
			System.out.println("initialized query object.");
			this.ruleStore = new de.dfki.lt.hfc.RuleStore(this.namespaces, this.tupleStore, ruleFile);
			System.out.println("loaded rule file.");

			//de.dfki.lt.hfc.Interactive intMode = new de.dfki.lt.hfc.Interactive(namespaceFile, tupleFile, ruleFile);
			
			this.forwardChainer = new de.dfki.lt.hfc.ForwardChainer(this.namespaces, this.tupleStore, this.ruleStore);
			System.out.println("created forward chainer object.");
			
			this.forwardChainer.computeClosure();
			System.out.println("Constructed a new HFCInterface!");
		}
		
		
		@Override
		public QueryResults querySelect(String q, Current current) {
			QueryResults _results = new QueryResults();
			
			if (q.toUpperCase().startsWith("SELECT")) {
				de.dfki.lt.hfc.BindingTable bt;
				_results.query = q;
				
				try {
					bt = this.query.query(q);
					if (bt == null) {
						System.out.println("HFC: query contains constants not known to the tuple store");
						_results.bt = new String[0][0];
						_results.varPosMap = new HashMap<String, Integer>();
					} 
					else {
						ArrayList<Map<String, String>> _hfcAnswer = 
							de.dfki.lt.hfc.BindingTableDecoder.decode(bt, this.tupleStore);
						System.out.println("received a decoded collection of size: " + _hfcAnswer.size());
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
					System.out.println("HFC: malformed SELECT query!");
					_results.bt = new String[0][0];
					_results.varPosMap = new HashMap<String, Integer>();
					e.printStackTrace();
				}
			}
			else {
				System.out.println("HFC: malformed SELECT query!");
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
