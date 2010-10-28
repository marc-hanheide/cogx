package coma.matrix;

import java.io.BufferedWriter;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import comadata.QueryResults;

import de.dfki.lt.hfc.*;

public class SimpleOMICSMatrix {

	public String namespaceFile="";
	public String tupleFile="";
	public String ruleFile = "";
	
	public String rowQuery = "";
	public String colQuery = "";

	private Namespace namespaces;
	private TupleStore tupleStore;
	private Query query;
	private RuleStore ruleStore;
	private ForwardChainer hfc;
	
	private Matrix<Boolean> matrix;
	
	public SimpleOMICSMatrix() {
		
	}
	
	public void init() {
		namespaces = new de.dfki.lt.hfc.Namespace(namespaceFile);
		tupleStore = new de.dfki.lt.hfc.TupleStore(namespaces, tupleFile);
		query = new de.dfki.lt.hfc.Query(tupleStore);
		ruleStore = new de.dfki.lt.hfc.RuleStore(namespaces, tupleStore, ruleFile);

		hfc = new de.dfki.lt.hfc.ForwardChainer(namespaces, tupleStore, ruleStore);
		hfc.computeClosure();

		System.out.println("created forward chainer object.");

		matrix = null;
	}
	
	public QueryResults querySelect(String q) {
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
	
	public void prepareMatrix() {
		String[] rowLabels;
		String[] colLabels;

	
		QueryResults rowResults = this.querySelect(this.rowQuery);
		ArrayList<String> __rowLabels = new ArrayList<String>();
		for (int i = 0; i < rowResults.bt.length; i++) {
			__rowLabels.add(rowResults.bt[i][0]);
		}
		rowLabels = new String[__rowLabels.size()]; 
		rowLabels = (String[]) __rowLabels.toArray(rowLabels);
		
		System.out.println("rowLabels[] has a length of " + rowLabels.length + "and the following entries:");
		for (int i = 0; i < rowLabels.length; i++) {
			System.out.println("rowLabels["+i+"] = " + rowLabels[i]);
		}
		
		QueryResults colResults = this.querySelect(this.colQuery);
		ArrayList<String> __colLabels = new ArrayList<String>();
		for (int i=0; i < colResults.bt.length; i++) {
			__colLabels.add(colResults.bt[i][0]);
		}
		colLabels = new String[__colLabels.size()];
		colLabels = (String[]) __colLabels.toArray(colLabels);
		
		try {
			this.matrix = new Matrix<Boolean>(colLabels, rowLabels, false);
		} catch (NonUniqueLabelException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void markCooccurrence(String coocQuery) {
		
		QueryResults qr = querySelect(coocQuery);
		for (int i=0; i<qr.bt.length ; i++) {
			String object = qr.bt[i][qr.varPosMap.get("?obj")];
			String location = qr.bt[i][qr.varPosMap.get("?loc")];
			System.out.println("Marking OMICS cooccurrence of " + object + " and " + location);
			this.matrix.insertCell(new Boolean(true), object, location);
		}
	
	}
	

	
	public static void main(String args[]) {
		SimpleOMICSMatrix my_matrix = new SimpleOMICSMatrix();
		
		my_matrix.namespaceFile = "subarchitectures/coma.sa/ontologies/dora.ns";
		my_matrix.tupleFile = "subarchitectures/coma.sa/ontologies/omics-locations-coocmatrix-subset1.nt";
		my_matrix.ruleFile = "subarchitectures/coma.sa/ontologies/default.rdl";
		
		my_matrix.init();
		
		my_matrix.colQuery = "SELECT DISTINCT ?obj where ?obj <http://dora.cogx.eu#in>  ?loc";
		my_matrix.rowQuery = "SELECT DISTINCT ?loc where ?obj <http://dora.cogx.eu#in>  ?loc";
		
		my_matrix.prepareMatrix();
		
		my_matrix.markCooccurrence("SELECT ?obj ?loc where ?obj <http://dora.cogx.eu#in>  ?loc");

		//try {
		//	my_matrix.matrix.saveToFile("simpleOmicsMatrix.mtx");
		//} catch (IOException e) {
		//	// TODO Auto-generated catch block
		//	e.printStackTrace();
		//}

		// save the matrix outcomes as nt file
		int numCols = my_matrix.matrix.getNumCols();
		int numRows = my_matrix.matrix.getNumRows();
		
		try{
		    // Create file 
		    FileOutputStream fstream = new FileOutputStream("subarchitectures/coma.sa/ontologies/tmp/omics-locations-coocmatrix.nt");
		    PrintStream outstream = new PrintStream(fstream);
		  //  BufferedWriter out = new BufferedWriter(fstream);
		        for (int a = 0; a < numCols ; a++) {
		        	for (int b = 0; b < numRows ; b++) {
		        		boolean _coocDefault = my_matrix.matrix.getCell(a, b);
		        		outstream.println(my_matrix.matrix.getColLabel(a) + 
		        				" <http://dora.cogx.eu#in> " + my_matrix.matrix.getRowLabel(b) +
		        				" " + (_coocDefault ? "\"0.7\"^^<xsd:float>" : "\"0.1\"^^<xsd:float>") +
		        				" .");
		        	}
		        }
		    outstream.close();
		    fstream.close();
		    }catch (Exception e){//Catch exception if any
		      System.err.println("Error: " + e.getMessage());
		    }
		System.out.println("Closed file streams.");
		
		System.exit(0);
		
	}

}
