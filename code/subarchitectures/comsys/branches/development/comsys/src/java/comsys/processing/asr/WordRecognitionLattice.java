
package comsys.processing.asr;


import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.Vector;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.StringTokenizer;

import opennlp.ccg.parse.CategoryChartScorer;
import opennlp.ccg.parse.FrontierCatFilter;
import opennlp.ccg.parse.UnrestrictedBeamWidthFunction;

import comsys.datastructs.comsysEssentials.PhonString;

import comsys.processing.parse.ActiveCCGLexicon;
import comsys.processing.parse.ActiveIncrCCGParser;
import comsys.processing.parse.PackedLFParseResults;
import opennlp.ccg.parse.ParseResults;

import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;

/**
 * A word recognition lattice builder from the ASR
 *
 * @author plison
 * @version 09/09/2008
 * 
 */
public class WordRecognitionLattice {

	Node startNode;
	Node endNode;
	
	Vector<Node> currentNodes;

	String Id;

	long maxLength = 0;

	static int idCount = 0;

	Vector<PhonString> rawresults;


	public WordRecognitionLattice(Vector<PhonString> lines, String Id) {
		this.Id = Id;
		rawresults = lines;
		for (Enumeration<PhonString> e = lines.elements(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			phon = smallChanges(phon);
			if (phon.length > maxLength) {
				maxLength = phon.length;
			}
		}

		startNode = extractLattice(lines);
		
		endNode.cleanPaths();
		
		currentNodes = new Vector<Node>();
		currentNodes.add(startNode);

	}
	
	public Vector<PhonString> getAllResults() {
		return rawresults;
	}
	
	public Vector<PhonString> getFirstResult() {
		Vector<PhonString> result = new Vector<PhonString>();
		result.add(rawresults.elementAt(0));
		return result;
	}
	
	public boolean isTraversalFinished() {
		if (currentNodes != null && currentNodes.size() == 1 
				&& currentNodes.elementAt(0).equals(endNode)) {
			return true;
		}
		return false;
	}


	private PhonString smallChanges(PhonString phon) {
		StringTokenizer tokenizer = new StringTokenizer (phon.wordSequence);
		String words = "";
		while (tokenizer.hasMoreTokens()) {
			String word = tokenizer.nextToken();
			if (word.equals("that's")) {
				word = "that is";
				phon.length++;
			}
			if (word.equals("don't")) {
				word = "do not";	
				phon.length++;
			}
			if (word.equals("it's")) {
				word = "it is";	
				phon.length++;
			}
			if (word.equals("where's")) {
				word = "where is";	
				phon.length++;
			}
			if (word.equals("i")) {
				word = "I";	
			}
			if (word.equals("no")) {
				word = "No";	
			}
			words += word;
			if (tokenizer.hasMoreTokens()) {
				words += " ";
			}
		}
		phon.wordSequence = words;
		return phon;
	}
	
	
	public Vector<PhonString> getNextResults() {
		Vector<PhonString> results = new Vector<PhonString>();
		Vector<Node> newnodes = new Vector<Node>();
		for (Enumeration<Node> c = currentNodes.elements() ; c.hasMoreElements();) {
			Node n = c.nextElement();
			for (Enumeration<Edge> d = n.outEdges.elements(); d.hasMoreElements();) {
				Edge edge = d.nextElement();
				Node n2 = edge.out;
				Vector<String> paths = n2.getBackwardPaths();
				for (Enumeration<String> e = paths.elements(); e.hasMoreElements();) {
					PhonString phon = new PhonString();
					phon.wordSequence = e.nextElement();
					results.add(phon);
				}
				newnodes.add(n2);
			}
		}
		currentNodes = newnodes;
		return results;
	}

	
	public Vector<PhonString> getFirstRawResult() {
		Vector<PhonString> result = new Vector<PhonString>();
		if (rawresults.size() > 0) {
			result.add(rawresults.elementAt(0));
			return result;
		}
		else
			return null;
	}

	public Vector<PhonString> getAllRawResults() {
		return rawresults;
	}

	public long getMaximumLength() {
		if (maxLength==0) {
			log("WARNING: length of the lattice is set to zero");
		}
		return maxLength;
	}

	public void storeResultsAsIs(String[] lines) {
		rawresults = new Vector<PhonString>();

		for (int i=0; i < lines.length ; i++) {
			String line = lines[i];
			String[] split = line.split(":");

			String recString;
			float confidence;
			if (split.length == 2) {
				PhonString phon = new PhonString();
				recString = split[0];
				confidence = new Float(split[1]).floatValue();
			}
			else {
				log("Confidence is not specified --> set to zero");
				recString = line;
				confidence = 0.0f;
			}
			PhonString phon = new PhonString();
			phon.wordSequence = recString;
			StringTokenizer tokenizer = new StringTokenizer(phon.wordSequence);
			phon.length = tokenizer.countTokens();
			if (phon.length > maxLength) {
				maxLength = phon.length;
			}
			phon.id = newId();
			phon.confidenceValue = confidence;
			rawresults.add(phon);
		}
	}


	public Node extractLattice(Vector<PhonString> phons) {

		startNode = new Node();
		endNode = new Node();
		
		Vector<StringTokenizer> tokenizers = new Vector<StringTokenizer>();

		for (Enumeration<PhonString> e = phons.elements(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			StringTokenizer tokenizer = new StringTokenizer(phon.wordSequence);
			tokenizers.add(tokenizer);
		}

		Node node =  extractLattice(tokenizers, new Vector<Node>());

		return node;
	}

	
	public Node extractLattice(Vector<StringTokenizer> tokenizers , Vector<Node> nodes) {
		
		Node curnode = new Node();

		Hashtable<String, Vector<StringTokenizer>> hash = 
			new Hashtable<String, Vector<StringTokenizer>>();

		for (Enumeration<StringTokenizer> e = tokenizers.elements(); e.hasMoreElements();) {
			StringTokenizer tokenizer = e.nextElement();
			if (tokenizer.hasMoreElements()) {
				String token = tokenizer.nextToken();
				if (hash.containsKey(token)) {
					Vector<StringTokenizer> existingtoks = hash.get(token);
					if (tokenizer.hasMoreElements()) {
						existingtoks.add(tokenizer);
						hash.put(token, existingtoks);
					}
					else {
						existingtoks.add(new StringTokenizer(""));
						hash.put(token, existingtoks);
					}

				}
				else {
					if (tokenizer.hasMoreElements()) {
						Vector<StringTokenizer> newtoks = new Vector<StringTokenizer>();
						newtoks.add(tokenizer);
						hash.put(token, newtoks);
					}
					else {
						Vector<StringTokenizer> newtoks = new Vector<StringTokenizer>();
						newtoks.add(new StringTokenizer(""));
						hash.put(token, newtoks);
					}					
				}
			}	
			else {
				return endNode;
			}
		}

		for (Enumeration<String> e = hash.keys(); e.hasMoreElements();) {
			String firstword = e.nextElement();
			Node newnode = extractLattice(hash.get(firstword), nodes);
			Node similarnode = null;
			for (Enumeration<Node> f = nodes.elements(); f.hasMoreElements();) {
				Node node = f.nextElement();
				if (node.similarPaths(newnode)) {
					similarnode = node;
				}
			}
			
			if (similarnode != null) {
				newnode = similarnode;
				Edge edge = new Edge(curnode, newnode, firstword, 0.0f);
				if (!curnode.outEdges.contains(edge)) {
					curnode.addOutEdge(edge);
					newnode.addInEdge(edge);
				}
			}
			else {
				Edge edge = new Edge(curnode, newnode, firstword, 0.0f);
		//		if (!nodes.contains(curnode))
		//			nodes.add(curnode);
		//		if (!nodes.contains(newnode))
					nodes.add(newnode);
				curnode.addOutEdge(edge);
				newnode.addInEdge(edge);
			}
		}
		return curnode;
	}

	public String getBestPath() {
		String result = "";
		return result;
	}

	public String toString() {
		String str = "";
		str += "Nuance Recognition results:\n" ;
		for (Enumeration<PhonString> e = rawresults.elements(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			str += phon.wordSequence + "\n";
		}
		return str;
	}

	public String newId() {
		idCount++;
		return "idphon-"+idCount;
	}

	static int NodeIncrement = 1;

	public final class Node {

		Vector<Edge> outEdges;
		Vector<Edge> inEdges;
		String nodeId;


		public Node() {
			inEdges = new Vector<Edge>();
			outEdges = new Vector<Edge>();
			nodeId = "node" + NodeIncrement;
			NodeIncrement++;
		}

		public void addOutEdge(Edge edge) {
			outEdges.add(edge);
		}
		
		public void addInEdge(Edge edge) {
			inEdges.add(edge);
		}

		public Vector<Edge> getOutEdges() {return outEdges; }
		
		public Vector<Edge> getInEdges() {return inEdges; }

		public boolean similarPaths(Node node) {
			Vector<String> ourpaths = getForwardPaths();
			Vector<String> theirpaths = node.getForwardPaths();
			if (ourpaths.size() != theirpaths.size()) {
				return false;
			}
			else {
				for (Enumeration<String> o = ourpaths.elements(); o.hasMoreElements();) {
					boolean found = false;
					String ourpath = o.nextElement();
					for (Enumeration<String> t = theirpaths.elements(); t.hasMoreElements();) {
						String theirpath = t.nextElement();
						if (ourpath.equals(theirpath)) {
							found = true;
						}
					}
					if (!found) {
						return false;
					}
				}
			}
			return true;
		}

		public Vector<String> getBackwardPaths() {
			Vector<String> results = new Vector<String>();
			for (Enumeration<Edge> e = inEdges.elements(); e.hasMoreElements();) {
				Edge edge = e.nextElement();
				Vector<String> previouspaths = edge.in.getBackwardPaths();
				if (previouspaths.size() == 0) {
					results.add(edge.label);
				}
				else {
					for (Enumeration<String> f = previouspaths.elements(); f.hasMoreElements();) {
						String previouspath = f.nextElement();
						String path = previouspath + " " + edge.label;
						results.add(path);
					}
				}
			}
			return results;
		}
	

		public Vector<String> getForwardPaths() {
			Vector<String> results = new Vector<String>();
			for (Enumeration<Edge> e = outEdges.elements(); e.hasMoreElements();) {
				Edge edge = e.nextElement();
				Vector<String> futurepaths = edge.out.getForwardPaths();
				if (futurepaths.size() == 0) {
					results.add(edge.label);
				}
				else {
					for (Enumeration<String> f = futurepaths.elements(); f.hasMoreElements();) {
						String futurepath = f.nextElement();
						String path = edge.label + " " + futurepath;
						results.add(path);
					}
				}
			}
			return results;
		}
		

		public void cleanPaths() {
			
			Vector<Edge> toRemove = new Vector<Edge>();
			for (Enumeration<Edge> e = inEdges.elements(); e.hasMoreElements();) {
				Edge edge = e.nextElement();
				
				if(edge.in.inEdges.size() == 0 && !edge.in.equals(startNode)) {
					edge.out.outEdges.remove(edge);
					toRemove.add(edge);
				}	
				edge.in.cleanPaths();
			}
			inEdges.removeAll(toRemove);
		}
	}



	public final class Edge {

		String label = "";
		float confidenceScore = 0.0f;
		Node in ;
		Node out;

		public Edge(Node in, Node out, String label, float confidenceScore) {
			this.in = in;
			this.out = out;
			this.label = label;
			this.confidenceScore = confidenceScore;
		}
		
		public boolean equals(Object other) {
			Edge otherEdge = (Edge) other;
			if (in.equals(otherEdge.in) && out.equals(otherEdge.out) 
					&& label.equals(otherEdge.label) && 
					confidenceScore == otherEdge.confidenceScore) {
				return true;
			}
			return false;
		}
	}


	public static void writeDOTFile(String DOTText, String DOTFile) {		
		try {
			FileOutputStream fout = new FileOutputStream (DOTFile);
			new PrintStream(fout).println (DOTText);
			fout.close();		
		}
		catch (java.io.IOException e) {
			e.printStackTrace();
		}
	}


	public String createDOTSpecs() {

		String text = "digraph G {\n";
		text += "rankdir=LR;\n";
		text += startNode.nodeId + " [label=\"\", shape=diamond,style=filled,color=lightgrey]\n";
		text += createSubDOTSpecsFromStart(startNode);
		text += "\n}";
		return text ;
	}
	

	public String createSubDOTSpecsFromEnd(Node node) {

		String text = "";
		if (node != null && node.getInEdges() != null && 
				node.getInEdges().size() > 0) {
			text += node.nodeId + " [label=\"\", shape=diamond,style=filled,color=lightgrey]\n";
			for (Enumeration<Edge> e = node.inEdges.elements(); e.hasMoreElements();) {
				Edge edge = e.nextElement();
				String newtext1 = createSubDOTSpecsFromEnd(edge.in);
				String[] split = newtext1.split("\n");
				for (int i = 0; i < split.length; i++) {
					if (!text.contains(split[i])) {
						text += split[i] + "\n";
					}
				}					
				String newText2= edge.in.nodeId  + " -> " + node.nodeId +  "[label=\"" + edge.label + "\"]\n";
				if (!text.contains(newText2)) {
					text += newText2;
				}
			}
		}
		return text ;
	}
	

	public String createSubDOTSpecsFromStart(Node node) {

		String text = "";
		if (node != null && node.getOutEdges() != null && 
				node.getOutEdges().size() > 0) {
			text += node.nodeId + " [label=\"\", shape=diamond,style=filled,color=lightgrey]\n";
			for (Enumeration<Edge> e = node.outEdges.elements(); e.hasMoreElements();) {
				Edge edge = e.nextElement();
				String newtext1 = createSubDOTSpecsFromStart(edge.out);
				String[] split = newtext1.split("\n");
				for (int i = 0; i < split.length; i++) {
					if (!text.contains(split[i])) {
						text += split[i] + "\n";
					}
				}					
				String newText2= node.nodeId  + " -> " + edge.out.nodeId +  "[label=\"" + edge.label + "\"]\n";
				if (!text.contains(newText2)) {
					text += newText2;
				}
			}
		}
		return text ;
	}


	public void latticeToGraph(String graphName, boolean generatePNG) {

		log ("Start generating the graphical version of the lattice ...");
		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecs() ;

		if (!DOTText.equals("")) {
			writeDOTFile(DOTText,DOTFile);

			if (generatePNG) {
				try	{
					Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
		log("File " + PNGFile + "successfully written");

	}



	public static void log(String str) {
		System.out.println("[lattice] " + str);
	}

	public String getId() { return Id; }


	public static void main(String[] args) {

		Vector<PhonString> phons = new Vector<PhonString>();
		PhonString phon1 = new PhonString();
		phon1.wordSequence = "get me the ball";
		phons.add(phon1);
		PhonString phon2 = new PhonString();
		phon2.wordSequence = "get me the table";
		phons.add(phon2);
		PhonString phon3 = new PhonString();
		phon3.wordSequence = "take the ball";
		phons.add(phon3);
		PhonString phon4 = new PhonString();
		phon4.wordSequence = "now get me the ball";
		phons.add(phon4);
		PhonString phon5 = new PhonString();
		phon5.wordSequence = "No robot";
		phons.add(phon5);
		PhonString phon6 = new PhonString();
		phon6.wordSequence = "aldldl";

		PhonString phon7 = new PhonString();
		phon7.wordSequence = "get me a ball";
		phons.add(phon7);
		
		WordRecognitionLattice lattice = new WordRecognitionLattice(phons, "id1");
		log(lattice.toString());
		lattice.latticeToGraph("lattice", true);
		

		for (Enumeration<PhonString> e = lattice.getNextResults().elements(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			log("nextresults1: " + phon.wordSequence);
		}
		
		for (Enumeration<PhonString> e = lattice.getNextResults().elements(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			log("nextresults2: " + phon.wordSequence);
		}
		
		ActiveIncrCCGParser parser = new ActiveIncrCCGParser();

		try {
			ActiveCCGLexicon grammar = new ActiveCCGLexicon();
			grammar.setGrammar("subarchitectures/comsys.mk4/grammars/openccg/robustmoloko/grammar.xml");
			parser.registerGrammarAccess(grammar);

			CategoryChartScorer cshs = new CategoryChartScorer();
			cshs.setBeamWidthFunction(new UnrestrictedBeamWidthFunction());
			parser.registerChartScorer(cshs);

			FrontierCatFilter cfilter = new FrontierCatFilter();
			parser.registerFrontierFilter(cfilter);

			/**
		PhonString phon1 = new PhonString();
		phon1.wordSequence = "get me the ball";
		phon1.id = "id1";
		PackedLFParseResults results1 = (PackedLFParseResults) parser.parse(phon1);
		results1.plf.packedLFId = "plf1";
		LFUtils.plfToGraph(results1.plf, "phon1", true);

		PhonString phon2 = new PhonString();
		phon2.wordSequence = "get me the table";
		phon2.id = "id2";
		PackedLFParseResults results2 = (PackedLFParseResults) parser.parse(phon2);
		LFUtils.plfToGraph(results2.plf, "phon2", true);
		results1.plf.packedLFId = "plf1";
			 */

			//	PackedLFParseResults results = (PackedLFParseResults) parser.parse(lattice);
			//	LFUtils.plfToGraph(results.plf, "lattice", true);

		}
		catch (Exception e) {
			e.printStackTrace();
		}

	}
}
