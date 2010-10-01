package coma.matrix;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;


import comadata.QueryResults;

import de.dfki.lt.hfc.*;

public class BingOMICSMatrix {
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

	private Matrix<Integer> matrix;

	private WebsiteScanner bingImagesScanner;
	
	public BingOMICSMatrix() {

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
		
		bingImagesScanner = new WebsiteScanner("span", "class", "sb_count");
		
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
			this.matrix = new Matrix<Integer>(colLabels, rowLabels, 0);
		} catch (NonUniqueLabelException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void markNumHits() {
		FileOutputStream fstream;
		try {
			DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-HH-mm");
	        Date date = new Date();
	        String currDateID = dateFormat.format(date);
	        
			fstream = new FileOutputStream("bing-omics-locations-coocmatrix."+currDateID+".nt");
			PrintStream outstream = new PrintStream(fstream);

			for (int a = 0; a < this.matrix.getNumCols() ; a++) {
				for (int b = 0; b < this.matrix.getNumRows() ; b++) {

					String object = this.matrix.getColLabel(a);
					String location = this.matrix.getRowLabel(b);

					String bingImagesQuery = "http://www.bing.com/images/search?q=%22"+object.split("#")[1].replace(">", "").replace("_", "+")+"%22+in+the+%22"+location.split("#")[1].replace(">", "").replace("_", "+")+"%22";

					System.out.println(bingImagesQuery);
					Integer numHits = bingImagesScanner.scanForHits(bingImagesQuery);

					System.out.println("Marking cooccurrence of " + object + " and " + location + " with number of Bing images hits: " + numHits);
					this.matrix.insertCell(numHits, a, b);

					outstream.println(object + 
							" <http://dora.cogx.eu/#in> " + location +
							" " + numHits);
				}
			}
			
			outstream.close();
			fstream.close();
			System.out.println("Closed file streams.");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}



	public static void main(String args[]) {
		// start in the dora directory using:
		// % java -cp output/classes/:hfc-hz.jar:./subarchitectures/coma.sa/lib/trove-2.1.0.jar:subarchitectures/coma.sa/lib/xercesImpl.jar -Xms800m -Xmx6200m  coma.matrix.BingOMICSMatrix

		
		BingOMICSMatrix my_matrix = new BingOMICSMatrix();

		my_matrix.namespaceFile = "dora.ns";
		my_matrix.tupleFile = "/Users/zender/projects/resources/omics/omics-locations.nt";
		my_matrix.ruleFile = "default.rdl";

		my_matrix.init();

		my_matrix.colQuery = "SELECT DISTINCT ?obj where ?obj <http://dora.cogx.eu/#omicsCooccurrence>  ?loc";
		my_matrix.rowQuery = "SELECT DISTINCT ?loc where ?obj <http://dora.cogx.eu/#omicsCooccurrence>  ?loc";

		my_matrix.prepareMatrix();

		my_matrix.markNumHits();
		// my_matrix.markCooccurrence("SELECT ?obj ?loc where ?obj <http://dora.cogx.eu/#omicsCooccurrence>  ?loc");

		//try {
		//	my_matrix.matrix.saveToFile("simpleOmicsMatrix.mtx");
		//} catch (IOException e) {
		//	// TODO Auto-generated catch block
		//	e.printStackTrace();
		//}

		// save the matrix outcomes as nt file
		//int numCols = my_matrix.matrix.getNumCols();
		//int numRows = my_matrix.matrix.getNumRows();

		//try{
			// Create file 
		//	FileOutputStream fstream = new FileOutputStream("bing-omics-locations-coocmatrix.nt");
		//	PrintStream outstream = new PrintStream(fstream);
		//	//  BufferedWriter out = new BufferedWriter(fstream);
		//	for (int a = 0; a < numCols ; a++) {
		//		for (int b = 0; b < numRows ; b++) {
		//			Integer numHits = getNumHits(my_matrix.matrix.getColLabel(a), my_matrix.matrix.getRowLabel(b));
		//				
		//			outstream.println(my_matrix.matrix.getColLabel(a) + 
		//					" <http://dora.cogx.eu/#in> " + my_matrix.matrix.getRowLabel(b) +
		//					" " + numHits);
		//		}
		//	}
		//	outstream.close();
		//	fstream.close();
		//}catch (Exception e){//Catch exception if any
		//	System.err.println("Error: " + e.getMessage());
		//}
		//System.out.println("Closed file streams.");

		System.exit(0);

	}

}
